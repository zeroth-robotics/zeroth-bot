use async_trait::async_trait;
use eyre::Result;
use kos_core::{
    google_proto::longrunning::Operation,
    hal::{Inference, InferenceState},
    kos_proto::{
        common::{ActionResponse, Error, ErrorCode},
        inference::*,
    },
};
use kinfer::{ModelRunner, MilkVModelRunner, kinfer_proto::ProtoIO};
use std::{sync::Arc, path::Path};
use tokio::sync::RwLock;
use tracing::{debug, error, info};

use crate::{actuator::{ZBotActuator, ZBOT_ALL_ACTUATOR_IDS}, imu::ZBotIMU};

const MILKV_STANDING_MODEL_PATH: &str = "/root/models/ppo_standing.cvimodel";

pub struct ZBotInference {
    model: Arc<RwLock<Option<MilkVModelRunner>>>,
    imu: Arc<ZBotIMU>,
    actuator: Arc<ZBotActuator>,
    state: Arc<RwLock<InferenceState>>,
}

impl ZBotInference {
    pub fn new(imu: Arc<ZBotIMU>, actuator: Arc<ZBotActuator>) -> Self {
        let model = match MilkVModelRunner::new(MILKV_STANDING_MODEL_PATH) {
            Ok(model) => Some(model),
            Err(e) => {
                error!("Failed to load default model: {}", e);
                None
            }
        };
        Self {
            model: Arc::new(RwLock::new(model)),
            imu,
            actuator,
            state: Arc::new(RwLock::new(InferenceState::Stopped)),
        }
    }

    async fn get_sensor_data(&self) -> Result<(IMUData, Vec<ActuatorStateResponse>)> {
        let imu_data = self.imu.get_values().await?;
        let actuator_data = self.actuator.get_actuators_state(ZBOT_ALL_ACTUATOR_IDS).await?;
        Ok((imu_data, actuator_data))
    }

    async fn pack_inputs_to_proto(
        &self,
        imu_data: IMUData,
        actuator_data: Vec<ActuatorStateResponse>,
    ) -> Result<ProtoIO> {
        let model = self.model.read().await;
        let model = model.as_ref().ok_or_else(|| eyre!("No model loaded"))?;
        
        let input_schema = model.input_schema()?;
        let mut proto_values = Vec::new();

        for value_schema in input_schema.values {
            match value_schema.value_type {
                Some(ValueType::JointPositions(ref joint_positions_schema)) => {
                    // Create position map for quick lookup from id
                    let position_map: HashMap<u32, f64> = actuator_data.iter()
                        .filter_map(|state| state.position.map(|pos| (state.actuator_id, pos)))
                        .collect();

                    // Pack joint positions into proto value with the proper order
                    let joint_positions = JointPositionsValue {
                        values: joint_positions_schema.joint_names.iter()
                            .map(|joint_name| {
                                let value = JOINT_NAME_TO_ID.get(joint_name)
                                    .and_then(|&id| position_map.get(&id))
                                    .map(|&pos| pos as f32)
                                    .unwrap_or(0.0);

                                JointPositionValue {
                                    joint_name: joint_name.clone(),
                                    value,
                                    unit: joint_positions_schema.unit, // Use schema-defined unit
                                }
                            })
                            .collect(),
                    };

                    let proto_value = ProtoValue {
                        value: Some(EnumValue::JointPositions(joint_positions))
                    };

                    proto_values.push((value_schema.value_name, proto_value));
                }
                Some(ValueType::Imu(ref imu_schema)) => {
                    let mut imu_values = Vec::new();
                    if imu_schema.use_accelerometer {
                        imu_values.extend_from_slice(&[
                            imu_data.accel_x as f32,
                            imu_data.accel_y as f32,
                            imu_data.accel_z as f32,
                        ]);
                    }
                    if imu_schema.use_gyroscope {
                        imu_values.extend_from_slice(&[
                            imu_data.gyro_x as f32,
                            imu_data.gyro_y as f32,
                            imu_data.gyro_z as f32,
                        ]);
                    }
                    if imu_schema.use_magnetometer {
                        imu_values.extend_from_slice(&[
                            imu_data.mag_x as f32,
                            imu_data.mag_y as f32,
                            imu_data.mag_z as f32,
                        ]);
                    }
                    
                    proto_values.push((value_schema.value_name, imu_values));
                },
                Some(ValueType::VectorCommand(ref vector_command_schema)) => {
                    // Default command vector of specified dimension
                    // TODO: replace with actual command vector (teleop?)
                    let command = vec![0.0f32; vector_command_schema.dimensions as usize];
                    proto_values.push((value_schema.value_name, command));
                },
                _ => {
                    error!("Unsupported input value type in schema");
                    return Err(eyre!("Unsupported input value type"));
                }
            }
        }

        Ok(ProtoIO {
            values: proto_values,
        })
    }

    async fn unpack_outputs_from_proto(&self, output: ProtoIO) -> Result<Vec<ActuatorCommand>> {
        let model = self.model.read().await;
        let model = model.as_ref().ok_or_else(|| eyre::eyre!("No model loaded"))?;

        let output_schema = model.output_schema()?;
        let mut actuator_commands = Vec::new();

        for (value_schema, proto_value) in output_schema.values.iter().zip(output.values.iter()) {
            match &value_schema.value_type {
                Some(ValueType::JointCommands(ref joint_commands_schema)) => {
                    if let Some(EnumValue::JointCommands(commands)) = &proto_value.value {
                        // Convert joint commands values to actuator commands
                        for joint_cmd in &commands.values {
                            // Look up actuator ID from joint name
                            if let Some(&actuator_id) = JOINT_NAME_TO_ID.get(&joint_cmd.joint_name) {
                                actuator_commands.push(ActuatorCommand {
                                    actuator_id,
                                    position: Some(joint_cmd.position as f64),
                                    velocity: None, // TODO: Velocity control not implemented yet
                                });
                            }
                        }
                    }
                }
                _ => {
                    error!("Unsupported output value type in schema");
                    return Err(eyre!("Unsupported output value type"));
                }
            }
        }

        Ok(actuator_commands)
    }

    async fn run_inference_cycle(&self) -> Result<()> {
        let model = self.model.read().await;
        let model = model.as_ref().ok_or_else(|| eyre::eyre!("No model loaded"))?;

        let (imu_data, actuator_data) = self.get_sensor_data().await?;
        
        let inputs = self.pack_inputs_to_proto(imu_data, actuator_data).await?;
        let outputs = model.run(inputs)?;
        let actuator_commands = self.unpack_outputs_from_proto(outputs).await?;

        self.actuator.command_actuators(actuator_commands).await?;

        Ok(())
    }
}

#[async_trait]
impl Inference for ZBotInference {
    async fn load_model(&self, path: String) -> Result<ActionResponse> {
        info!("Loading model from: {}", path);
        
        let model = match MilkVModelRunner::new(Path::new(&path)) {
            Ok(model) => model,
            Err(e) => {
                error!("Failed to load model: {}", e);
                return Ok(ActionResponse {
                    success: false,
                    error: Some(Error {
                        code: ErrorCode::InvalidArgument as i32,
                        message: format!("Failed to load model: {}", e),
                    }),
                });
            }
        };

        let mut model_guard = self.model.write().await;
        *model_guard = Some(model);

        Ok(ActionResponse {
            success: true,
            error: None,
        })
    }

    async fn start(&self) -> Result<ActionResponse> {
        let mut state = self.state.write().await;
        
        if *state == InferenceState::Running {
            return Ok(ActionResponse {
                success: false,
                error: Some(Error {
                    code: ErrorCode::AlreadyExists as i32,
                    message: "Inference is already running".to_string(),
                }),
            });
        }

        // Start inference loop in background task
        let self_clone = Arc::new(self.clone());
        tokio::spawn(async move {
            loop {
                let state = self_clone.state.read().await;
                if *state != InferenceState::Running {
                    break;
                }
                
                if let Err(e) = self_clone.run_inference_cycle().await {
                    error!("Inference cycle failed: {}", e);
                }
                
                tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            }
        });

        *state = InferenceState::Running;

        Ok(ActionResponse {
            success: true,
            error: None,
        })
    }

    async fn stop(&self) -> Result<ActionResponse> {
        let mut state = self.state.write().await;
        *state = InferenceState::Stopped;

        Ok(ActionResponse {
            success: true,
            error: None,
        })
    }

    async fn get_state(&self) -> Result<InferenceState> {
        Ok(*self.state.read().await)
    }
}