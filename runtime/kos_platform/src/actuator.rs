use crate::firmware::hal::{
    ServoDirection, ServoRegister, TorqueMode,
    ServoMultipleWriteCommand, MAX_SERVOS
};
use crate::Servo;
use eyre::Result;
use kos_core::google_proto::longrunning::Operation;
use kos_core::hal::Actuator;
use kos_core::kos_proto::actuator::*;
use kos_core::kos_proto::common::{ActionResponse, ActionResult, Error as KosError, ErrorCode};
use std::sync::{Arc, Mutex};
use tokio::sync::RwLock;
use tonic::Status;

pub struct ZBotActuator {
    servo: Arc<Mutex<Servo>>,
    enabled_servos: Arc<RwLock<Vec<u32>>>,
}

impl ZBotActuator {
    pub async fn new() -> Result<Self> {
        let servo = Servo::new()?;
        servo.enable_readout()?;
        Ok(Self {
            servo: Arc::new(Mutex::new(servo)),
            enabled_servos: Arc::new(RwLock::new(Vec::new())),
        })
    }
}

#[tonic::async_trait]
impl Actuator for ZBotActuator {
    async fn command_actuators(&self, commands: Vec<ActuatorCommand>) -> Result<Vec<ActionResult>> {
        // First get read access to enabled_servos
        let enabled_servos = self.enabled_servos.read().await;
        
        // Then do the servo operations in a separate block so MutexGuard is dropped
        let result = {
            let servo = self.servo.lock().map_err(|e| eyre::eyre!("Lock error: {}", e))?;
            
            let mut cmd = ServoMultipleWriteCommand {
                only_write_positions: 1,
                ids: [253; MAX_SERVOS], // Initialize with 253 (disabled)
                positions: [0; MAX_SERVOS],
                times: [0; MAX_SERVOS],
                speeds: [0; MAX_SERVOS],
            };

            let mut results = Vec::new();
            for command in &commands {
                // Skip if servo isn't enabled
                if !enabled_servos.contains(&command.actuator_id) {
                    results.push(ActionResult {
                        actuator_id: command.actuator_id,
                        success: false,
                        error: Some(KosError {
                            code: ErrorCode::InvalidArgument as i32,
                            message: "Servo not enabled".to_string(),
                        }),
                    });
                    continue;
                }

                let idx = (command.actuator_id - 1) as usize;
                if idx >= MAX_SERVOS {
                    results.push(ActionResult {
                        actuator_id: command.actuator_id,
                        success: false,
                        error: Some(KosError {
                            code: ErrorCode::InvalidArgument as i32,
                            message: "Invalid servo ID".to_string(),
                        }),
                    });
                    continue;
                }

                cmd.ids[idx] = command.actuator_id as u8;

                if let Some(position) = command.position {
                    cmd.positions[idx] = Servo::degrees_to_raw(position as f32) as i16;
                } else if let Some(velocity) = command.velocity {
                    let speed = velocity.abs() as u16;
                    let direction = if velocity >= 0.0 {
                        ServoDirection::Clockwise
                    } else {
                        ServoDirection::Counterclockwise
                    };
                    // Set speed for individual servo since it can't be batched
                    if let Err(e) = servo.set_speed(command.actuator_id as u8, speed, direction) {
                        results.push(ActionResult {
                            actuator_id: command.actuator_id,
                            success: false,
                            error: Some(KosError {
                                code: ErrorCode::HardwareFailure as i32,
                                message: e.to_string(),
                            }),
                        });
                        continue;
                    }
                }

                results.push(ActionResult {
                    actuator_id: command.actuator_id,
                    success: true,
                    error: None,
                });
            }

            // Execute batch command if any positions were set
            if results.iter().any(|r| r.success) {
                if let Err(e) = servo.write_multiple(&cmd) {
                    let failed_results = results.into_iter().map(|mut r| {
                        if r.success {
                            r.success = false;
                            r.error = Some(KosError {
                                code: ErrorCode::HardwareFailure as i32,
                                message: e.to_string(),
                            });
                        }
                        r
                    }).collect();
                    return Ok::<Vec<ActionResult>, eyre::Report>(failed_results);
                }
            }

            Ok::<Vec<ActionResult>, eyre::Report>(results)
        }?;

        Ok(result)
    }

    async fn configure_actuator(&self, config: ConfigureActuatorRequest) -> Result<ActionResponse> {
        // First get write access to enabled_servos
        let mut enabled_servos = self.enabled_servos.write().await;
        
        // Then do the servo operations in a separate block so MutexGuard is dropped
        let result = {
            let servo = self.servo.lock().map_err(|_| Status::internal("Lock error"))?;

            // Unlock EEPROM for writing
            servo.write(config.actuator_id as u8, ServoRegister::LockMark, &[0])
                .map_err(|e| Status::internal(e.to_string()))?;

            let mut result = Ok(());

            // Apply configurations
            if let Some(kp) = config.kp {
                result = result.and(servo.write(
                    config.actuator_id as u8,
                    ServoRegister::PProportionalCoeff,
                    &[kp as u8],
                ));
            }
            if let Some(ki) = config.ki {
                result = result.and(servo.write(
                    config.actuator_id as u8,
                    ServoRegister::IIntegralCoeff,
                    &[ki as u8],
                ));
            }
            if let Some(kd) = config.kd {
                result = result.and(servo.write(
                    config.actuator_id as u8,
                    ServoRegister::DDifferentialCoeff,
                    &[kd as u8],
                ));
            }
            if let Some(torque_enabled) = config.torque_enabled {
                let mode = if torque_enabled {
                    TorqueMode::Enabled
                } else {
                    TorqueMode::Disabled
                };
                result = result.and(servo.set_torque_mode(config.actuator_id as u8, mode));
                
                // Update enabled servos list
                if torque_enabled {
                    if !enabled_servos.contains(&config.actuator_id) {
                        enabled_servos.push(config.actuator_id);
                    }
                } else {
                    enabled_servos.retain(|&id| id != config.actuator_id);
                }
            }

            // Lock EEPROM after writing
            servo.write(config.actuator_id as u8, ServoRegister::LockMark, &[1])
                .map_err(|e| Status::internal(e.to_string()))?;

            result
        };

        match result {
            Ok(_) => Ok(ActionResponse {
                success: true,
                error: None,
            }),
            Err(e) => Ok(ActionResponse {
                success: false,
                error: Some(KosError {
                    code: ErrorCode::HardwareFailure as i32,
                    message: e.to_string(),
                }),
            }),
        }
    }

    async fn calibrate_actuator(&self, _request: CalibrateActuatorRequest) -> Result<Operation> {
        Ok(Operation::default())
    }

    async fn get_actuators_state(
        &self,
        actuator_ids: Vec<u32>,
    ) -> Result<Vec<ActuatorStateResponse>> {
        let servo = self.servo.lock().map_err(|_| Status::internal("Lock error"))?;
        
        // Read all servo data in one batch
        let servo_data = servo.read_continuous()
            .map_err(|e| Status::internal(e.to_string()))?;

        let mut states = Vec::new();
        for id in actuator_ids {
            let idx = (id - 1) as usize;
            if idx >= MAX_SERVOS {
                states.push(ActuatorStateResponse {
                    actuator_id: id,
                    online: false,
                    position: None,
                    velocity: None,
                    torque: None,
                    temperature: None,
                    voltage: None,
                    current: None,
                });
                continue;
            }

            let info = &servo_data.servo[idx];
            
            // Check if servo is responding (using voltage as indicator)
            if info.current_voltage != 0 {
                states.push(ActuatorStateResponse {
                    actuator_id: id,
                    online: true,
                    position: Some(Servo::raw_to_degrees(info.current_location as u16) as f64),
                    velocity: Some({
                        let speed_raw = info.current_speed as u16;
                        let speed_magnitude = speed_raw & 0x7FFF;
                        let speed_sign = if speed_raw & 0x8000 != 0 { -1.0 } else { 1.0 };
                        speed_sign * (speed_magnitude as f32 * 360.0 / 4096.0) as f64
                    }),
                    torque: Some({
                        let torque_enabled = info.torque_switch != 0;
                        let torque_limit = info.torque_limit as f32 / 1000.0; // Convert to 0-1 range
                        if torque_enabled { torque_limit } else { 0.0 }
                    } as f64),
                    temperature: Some(info.current_temperature as f64),
                    voltage: Some(info.current_voltage as f32 / 10.0), // Convert to volts
                    current: Some(info.current_current as f32 / 100.0), // Convert to amps
                });
            } else {
                states.push(ActuatorStateResponse {
                    actuator_id: id,
                    online: false,
                    position: None,
                    velocity: None,
                    torque: None,
                    temperature: None,
                    voltage: None,
                    current: None,
                });
            }
        }

        Ok(states)
    }
}
