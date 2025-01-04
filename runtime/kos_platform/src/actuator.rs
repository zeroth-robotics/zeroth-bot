use crate::firmware::hal::{ServoDirection, ServoMode, ServoRegister, TorqueMode};
use crate::Servo;
use eyre::Result;
use kos_core::google_proto::longrunning::Operation;
use kos_core::hal::Actuator;
use kos_core::kos_proto::actuator::*;
use kos_core::kos_proto::common::{ActionResponse, ActionResult, Error as KosError, ErrorCode};
use std::sync::{Arc, Mutex};
use tokio::sync::RwLock;
use tonic::{Request, Response, Status};

pub struct ZBotActuator {
    servo: Arc<Mutex<Servo>>,
}

impl ZBotActuator {
    pub async fn new() -> Result<Self> {
        let servo = Servo::new()?;
        servo.enable_readout()?;
        Ok(Self {
            servo: Arc::new(Mutex::new(servo)),
        })
    }
}

#[tonic::async_trait]
impl Actuator for ZBotActuator {
    async fn command_actuators(&self, commands: Vec<ActuatorCommand>) -> Result<Vec<ActionResult>> {
        let servo = self
            .servo
            .lock()
            .map_err(|_| Status::internal("Lock error"))?;

        let mut results = Vec::new();
        for cmd in commands {
            let result = if let Some(position) = cmd.position {
                // Convert degrees to raw servo position
                let raw_position = Servo::degrees_to_raw(position as f32);
                servo.move_servo(cmd.actuator_id as u8, raw_position as i16, 0, 0)
            } else if let Some(velocity) = cmd.velocity {
                // Set speed and direction
                let speed = velocity.abs() as u16;
                let direction = if velocity >= 0.0 {
                    ServoDirection::Clockwise
                } else {
                    ServoDirection::Counterclockwise
                };
                servo.set_speed(cmd.actuator_id as u8, speed, direction)
            } else {
                Ok(()) // No command specified
            };

            let success = result.is_ok();
            let error = result.err().map(|e| KosError {
                code: ErrorCode::HardwareFailure as i32,
                message: e.to_string(),
            });

            results.push(ActionResult {
                actuator_id: cmd.actuator_id,
                success,
                error,
            });
        }

        Ok(results)
    }

    async fn configure_actuator(&self, config: ConfigureActuatorRequest) -> Result<ActionResponse> {
        let servo = self
            .servo
            .lock()
            .map_err(|_| Status::internal("Lock error"))?;

        // Unlock EEPROM for writing
        servo
            .write(config.actuator_id as u8, ServoRegister::LockMark, &[0])
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
        }

        // Lock EEPROM after writing
        servo
            .write(config.actuator_id as u8, ServoRegister::LockMark, &[1])
            .map_err(|e| Status::internal(e.to_string()))?;

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

    async fn calibrate_actuator(&self, request: CalibrateActuatorRequest) -> Result<Operation> {
        Ok(Operation::default())
    }

    async fn get_actuators_state(
        &self,
        actuator_ids: Vec<u32>,
    ) -> Result<Vec<ActuatorStateResponse>> {
        let servo = self
            .servo
            .lock()
            .map_err(|_| Status::internal("Lock error"))?;

        let mut states = Vec::new();
        for id in actuator_ids {
            if let Ok(info) = servo.read_info(id as u8) {
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
                    torque: None,
                    temperature: Some(info.current_temperature as f64),
                    voltage: Some(info.current_voltage as f32 / 10.0),
                    current: Some(info.current_current as f32 / 100.0),
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
