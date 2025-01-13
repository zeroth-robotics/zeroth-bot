use crate::firmware::feetech::{
    FeetechActuator, FeetechActuatorInfo, FeetechActuatorType, FeetechSupervisor,
};
use eyre::Result;
use kos_core::google_proto::longrunning::Operation;
use kos_core::hal::Actuator;
use kos_core::kos_proto::actuator::*;
use kos_core::kos_proto::common::{ActionResponse, ActionResult, Error as KosError, ErrorCode};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use tokio::sync::RwLock;
use tonic::{Request, Response, Status};
use lazy_static::lazy_static;

pub struct ZBotActuator {
    supervisor: Arc<RwLock<FeetechSupervisor>>,
}
pub const ZBOT_ALL_ACTUATOR_IDS: [u32; 16] = [1, 2, 3, 4, 5,
                                   6, 7, 8, 9, 10,
                                   11, 12, 13,
                                   14, 15, 16];

                                   
lazy_static! {
    pub static ref JOINT_NAME_TO_ID: HashMap<String, u32> = {
        let mut map = HashMap::new();
        // Right leg
        map.insert("right_ankle_pitch".to_string(), 1);
        map.insert("right_knee_pitch".to_string(), 2);
        map.insert("right_hip_roll".to_string(), 3);
        map.insert("right_hip_yaw".to_string(), 4);
        map.insert("right_hip_pitch".to_string(), 5);
        
        // Left leg
        map.insert("left_ankle_pitch".to_string(), 6);
        map.insert("left_knee_pitch".to_string(), 7);
        map.insert("left_hip_roll".to_string(), 8);
        map.insert("left_hip_yaw".to_string(), 9);
        map.insert("left_hip_pitch".to_string(), 10);
        
        // Right arm
        map.insert("right_elbow_yaw".to_string(), 11);
        map.insert("right_shoulder_yaw".to_string(), 12);
        map.insert("right_shoulder_pitch".to_string(), 13);
        
        // Left arm
        map.insert("left_shoulder_pitch".to_string(), 14);
        map.insert("left_shoulder_yaw".to_string(), 15);
        map.insert("left_elbow_yaw".to_string(), 16);
        
        map
    };

    pub static ref ID_TO_JOINT_NAME: HashMap<u32, String> = {
        let mut map = HashMap::new();
        for (name, &id) in JOINT_NAME_TO_ID.iter() {
            map.insert(id, name.clone());
        }
        map
    };
}

impl ZBotActuator {
    pub async fn new() -> Result<Self> {
        let mut supervisor = FeetechSupervisor::new()?;

        // Add the servos
        for id in ZBOT_ALL_ACTUATOR_IDS {
            supervisor
                .add_servo(id as u8, FeetechActuatorType::Sts3215)
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
                    servo.set_pid(p, i, d);
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
                    online: true,
                    position: Some(info.position_deg as f64),
                    velocity: Some(info.speed_deg_per_s as f64),
                    torque: Some(info.load_percent as f64),
                    temperature: Some(info.temperature_c as f64),
                    voltage: Some(info.voltage_v),
                    current: Some(info.current_ma), // Convert mA to A
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

    async fn calibrate_actuator(&self, request: CalibrateActuatorRequest) -> Result<Operation> {
        Ok(Operation::default())
    }
}
