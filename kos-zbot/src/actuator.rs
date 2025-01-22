use crate::firmware::feetech::{FeetechActuatorType, FeetechSupervisor};
use eyre::Result;
use kos::hal::{Actuator, Operation};
use kos::kos_proto::{
    actuator::*,
    common::{ActionResponse, ActionResult, Error as KosError, ErrorCode},
};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

pub struct ZBotActuator {
    supervisor: Arc<RwLock<FeetechSupervisor>>,
}

impl ZBotActuator {
    pub async fn new(actuator_list: &[u8]) -> Result<Self> {
        let mut supervisor = FeetechSupervisor::new()?;

        for id in actuator_list {
            supervisor
                .add_servo(*id, FeetechActuatorType::Sts3215)
                .await?;
        }

        Ok(Self {
            supervisor: Arc::new(RwLock::new(supervisor)),
        })
    }
}

#[tonic::async_trait]
impl Actuator for ZBotActuator {
    async fn command_actuators(&self, commands: Vec<ActuatorCommand>) -> Result<Vec<ActionResult>> {
        let mut supervisor = self.supervisor.write().await;
        let mut desired_positions = HashMap::new();

        let mut results = Vec::new();
        for cmd in commands {
            let result = if let Some(position) = cmd.position {
                desired_positions.insert(cmd.actuator_id as u8, position as f32);
                Ok(())
            } else if let Some(_velocity) = cmd.velocity {
                // Velocity control not implemented yet in the new library
                Err(eyre::eyre!("Velocity control not implemented"))
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

        if !desired_positions.is_empty() {
            supervisor.move_actuators(&desired_positions).await?;
        }

        Ok(results)
    }

    async fn configure_actuator(&self, config: ConfigureActuatorRequest) -> Result<ActionResponse> {
        let mut supervisor = self.supervisor.write().await;

        let result = {
            let id = config.actuator_id as u8;

            if let Some(torque_enabled) = config.torque_enabled {
                if torque_enabled {
                    supervisor.enable_torque(id).await?;
                } else {
                    supervisor.disable_torque(id).await?;
                }
            }

            if let Some(new_actuator_id) = config.new_actuator_id {
                supervisor.change_id(id, new_actuator_id as u8).await?;
            }

            let mut servos = supervisor.servos.write().await;
            if let Some(servo) = servos.get_mut(&id) {
                // Set PID values if provided
                let p = config.kp.map(|v| v as f32);
                let i = config.ki.map(|v| v as f32);
                let d = config.kd.map(|v| v as f32);
                if p.is_some() || i.is_some() || d.is_some() {
                    let _ = servo.set_pid(p, i, d);
                }

                if let Some(zero_position) = config.zero_position {
                    if zero_position {
                        servo.set_zero_position()?;
                    }
                }
                Ok(())
            } else {
                Err(eyre::eyre!("Servo not found"))
            }
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

    async fn get_actuators_state(
        &self,
        actuator_ids: Vec<u32>,
    ) -> Result<Vec<ActuatorStateResponse>> {
        let supervisor = self.supervisor.read().await;
        let servos = supervisor.servos.read().await;

        let mut states = Vec::new();
        for id in actuator_ids {
            if let Some(servo) = servos.get(&(id as u8)) {
                let info = servo.info();
                states.push(ActuatorStateResponse {
                    actuator_id: id,
                    online: info.online,
                    position: Some(info.position_deg as f64),
                    velocity: Some(info.speed_deg_per_s as f64),
                    torque: Some(info.load_percent as f64),
                    temperature: Some(info.temperature_c as f64),
                    voltage: Some(info.voltage_v),
                    current: Some(info.current_ma), // Convert mA to A
                    faults: info.faults,
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
                    faults: Vec::new(),
                });
            }
        }

        Ok(states)
    }

    async fn calibrate_actuator(&self, _request: CalibrateActuatorRequest) -> Result<Operation> {
        Ok(Operation::default())
    }
}
