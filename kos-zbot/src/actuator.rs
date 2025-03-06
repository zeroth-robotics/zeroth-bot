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
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use tracing::{debug,warn};

pub struct ZBotActuator {
    supervisor: Arc<RwLock<FeetechSupervisor>>,
    command_rate: f64, // Hz
    desired_positions: Arc<RwLock<HashMap<u8, f32>>>,
    desired_velocities: Arc<RwLock<HashMap<u8, f32>>>,
    command_task_running: Arc<AtomicBool>,
}

impl ZBotActuator {
    pub async fn new(actuator_list: &[u8]) -> Result<Self> {
        let mut supervisor = FeetechSupervisor::new()?;

        for id in actuator_list {
            supervisor.add_servo(*id, FeetechActuatorType::Sts3215).await?;
        }

        Ok(Self {
            supervisor: Arc::new(RwLock::new(supervisor)),
            command_rate: 50.0, // Default 50Hz
            desired_positions: Arc::new(RwLock::new(HashMap::new())),
            desired_velocities: Arc::new(RwLock::new(HashMap::new())),
            command_task_running: Arc::new(AtomicBool::new(false)),
        })
    }

    fn start_command_task(&self) {
        if self.command_task_running.load(Ordering::SeqCst) {
            return; // Task already running
        }

        let supervisor = self.supervisor.clone();
        let desired_positions = self.desired_positions.clone();
        let desired_velocities = self.desired_velocities.clone();
        let command_task_running = self.command_task_running.clone();
        let period = Duration::from_secs_f64(1.0 / self.command_rate);

        command_task_running.store(true, Ordering::SeqCst);

        tokio::spawn(async move {
            let mut interval = tokio::time::interval(period);

            while command_task_running.load(Ordering::SeqCst) {
                interval.tick().await;
                
                let positions = desired_positions.read().await.clone();
                let velocities = desired_velocities.read().await.clone();

                if !positions.is_empty() || !velocities.is_empty() {
                    if let Err(e) = supervisor.write().await.move_actuators(&positions, &velocities).await {
                        warn!("Failed to command actuators: {}", e);
                    }
                }
            }
        });
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
        let mut results = Vec::new();
        let mut positions = self.desired_positions.write().await;
        let mut velocities = self.desired_velocities.write().await;

        for cmd in commands {
            // Track if we should remove this actuator from continuous command
            let mut remove_actuator = true;
            
            if let Some(position) = cmd.position {
                positions.insert(cmd.actuator_id as u8, position as f32);
                remove_actuator = false;
            }
            
            if let Some(velocity) = cmd.velocity {
                velocities.insert(cmd.actuator_id as u8, velocity as f32);
                remove_actuator = false;
            }

            // If neither position nor velocity was specified, remove the actuator from continuous command
            if remove_actuator {
                positions.remove(&(cmd.actuator_id as u8));
                velocities.remove(&(cmd.actuator_id as u8));
            }

            results.push(ActionResult {
                actuator_id: cmd.actuator_id,
                success: true,
                error: None,
            });
        }

        // Start the command task if we have any positions or velocities to command
        if !positions.is_empty() || !velocities.is_empty() {
            self.start_command_task();
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
