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
use tracing::{debug};

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

    fn to_action_response<E: std::fmt::Display>(result: Result<(), E>) -> ActionResponse {
        match result {
            Ok(()) => ActionResponse {
                success: true,
                error: None,
            },
            Err(e) => ActionResponse {
                success: false,
                error: Some(KosError {
                    code: ErrorCode::HardwareFailure as i32,
                    message: e.to_string(),
                }),
            },
        }
    }

    fn success_response() -> ActionResponse {
        ActionResponse {
            success: true,
            error: None,
        }
    }
}

#[tonic::async_trait]
impl Actuator for ZBotActuator {
    async fn command_actuators(&self, commands: Vec<ActuatorCommand>) -> Result<Vec<ActionResult>> {
        let mut supervisor = self.supervisor.write().await;
        let mut desired_positions = HashMap::new();
        let mut desired_velocities = HashMap::new();
        //let mut desired_time = HashMap::new();

        let mut results = Vec::new();
        
        for cmd in commands {
            let result = Ok(());

            if let Some(position) = cmd.position {
                desired_positions.insert(cmd.actuator_id as u8, position as f32);
            }
            
            /*if let Some(time) = cmd.time {
                desired_time.insert(cmd.actuator_id as u8, time as f32);
            }*/

            if let Some(velocity) = cmd.velocity {
                desired_velocities.insert(cmd.actuator_id as u8, velocity as f32);
            }

            let success = result.is_ok();
            let error = result.err().map(|e: eyre::Error| KosError {
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
            //supervisor.move_actuators(&desired_positions, &desired_time, &desired_velocities).await?;
            supervisor.move_actuators(&desired_positions, &desired_velocities).await?;
        }

        Ok(results)
    }

    async fn configure_actuator(&self, config: ConfigureActuatorRequest) -> Result<ActionResponse> {
        let mut supervisor = self.supervisor.write().await;
        let id = config.actuator_id as u8;
        let mut errors = Vec::new();
        debug!("configure_actuator [id]:{}", id);
        
        {   // <-- block to limit servo lock life
            let mut servos = supervisor.servos.write().await;
            if let Some(servo) = servos.get_mut(&id) {
                // Set PID values if provided.
                let p = config.kp.map(|v| v as f32);
                let i = config.ki.map(|v| v as f32);
                let d = config.kd.map(|v| v as f32);
                if p.is_some() || i.is_some() || d.is_some() {
                    if let Err(e) = servo.set_pid(p, i, d) {
                        errors.push(e.into());
                    }
                }

                if let Some(acceleration) = config.acceleration {
                    if let Err(e) = servo.set_acceleration(acceleration as f32) {
                        errors.push(e.into());
                    }
                }

                if let Some(zero_position) = config.zero_position {
                    debug!("zero position");
                    if zero_position {
                        if let Err(e) = servo.set_zero_position() {
                            errors.push(e.into());
                        }
                    }
                }
            } else {
                return Ok(Self::to_action_response(Err(eyre::eyre!("Servo not found"))));
            }
        } // <-- servo lock dropoed>
    
        if let Some(torque_enabled) = config.torque_enabled {
            let result = if torque_enabled {
                supervisor.enable_torque(id).await
            } else {
                supervisor.disable_torque(id).await
            };
            if let Err(e) = result {
                errors.push(e);
            }
        }
        
        if let Some(new_actuator_id) = config.new_actuator_id {
            let result = supervisor.change_id(id, new_actuator_id as u8).await;
            if let Err(e) = result {
                errors.push(e);
            }
        }
        
        // Return an aggregated response.
        if errors.is_empty() {
            Ok(Self::success_response())
        } else {
            Ok(Self::to_action_response(Err(errors.remove(0))))
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
