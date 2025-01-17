use crate::Model;
use eyre::Result;
use kos::hal::Inference;
use kos::kos_proto::common::ErrorCode;
use kos::kos_proto::common::{ActionResponse, Error};
use kos::kos_proto::inference::get_models_info_request::Filter;
use kos::kos_proto::inference::ModelUids;
use kos::kos_proto::inference::*;
use kos::kos_proto::inference::Tensor as ProtoTensor;
use kos::kos_proto::inference::tensor::Dimension as ProtoDimension;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::Path;
use std::path::PathBuf;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{debug, error, info};
use uuid::Uuid;
use crate::Model as CvitekModel;

const MODELS_DIR: &'static str = "/opt/models";
const METADATA_FILE: &'static str = "/opt/models/metadata.json";

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SerializableModelMetadata {
    pub model_name: Option<String>,
    pub model_description: Option<String>,
    pub model_version: Option<String>,
    pub model_author: Option<String>,
}

impl From<&ModelMetadata> for SerializableModelMetadata {
    fn from(metadata: &ModelMetadata) -> Self {
        Self {
            model_name: metadata.model_name.clone(),
            model_description: metadata.model_description.clone(),
            model_version: metadata.model_version.clone(),
            model_author: metadata.model_author.clone(),
        }
    }
}

impl From<SerializableModelMetadata> for ModelMetadata {
    fn from(metadata: SerializableModelMetadata) -> Self {
        Self {
            model_name: metadata.model_name,
            model_description: metadata.model_description,
            model_version: metadata.model_version,
            model_author: metadata.model_author,
        }
    }
}

impl Default for SerializableModelMetadata {
    fn default() -> Self {
        Self {
            model_name: None,
            model_description: None,
            model_version: None,
            model_author: None,
        }
    }
}

pub struct ZBotInference {
    loaded_models: Arc<RwLock<HashMap<String, CvitekModel>>>,
    available_models: Arc<RwLock<HashMap<String, SerializableModelMetadata>>>,
}

impl ZBotInference {
    pub fn new() -> Result<Self> {
        let inference = Self {
            loaded_models: Arc::new(RwLock::new(HashMap::new())),
            available_models: Arc::new(RwLock::new(HashMap::new())),
        };

        // Load metadata directly
        tokio::task::block_in_place(|| {
            tokio::runtime::Handle::current().block_on(async {
                if let Err(e) = inference.load_metadata().await {
                    error!("Failed to load metadata: {}", e);
                }
            })
        });

        Ok(inference)
    }

    fn generate_model_uid() -> String {
        Uuid::new_v4().to_string()
    }

    async fn register_model(&self, model_uid: String, metadata: ModelMetadata) -> Result<()> {
        let serializable_metadata = SerializableModelMetadata::from(&metadata);
        let mut all_models = self.available_models.write().await;
        all_models.insert(model_uid.clone(), serializable_metadata);
        drop(all_models);

        self.save_metadata().await?;
        Ok(())
    }

    async fn save_model_binary(&self, model_data: Vec<u8>) -> Result<PathBuf> {
        let models_dir = PathBuf::from(MODELS_DIR);

        // Create models directory with detailed error handling
        if let Err(e) = std::fs::create_dir_all(&models_dir) {
            error!(
                "Failed to create models directory at {}: {}",
                models_dir.display(),
                e
            );
            return Err(eyre::eyre!(
                "Failed to create models directory: {}. Check permissions and disk space",
                e
            ));
        }

        // Check available disk space (require at least model size + 10MB buffer)
        let required_space = model_data.len() as u64 + 10 * 1024 * 1024; // model size + 10MB
        if let Ok(space) = fs2::available_space(&models_dir) {
            if space < required_space {
                error!(
                    "Insufficient disk space. Required: {}MB, Available: {}MB",
                    required_space / (1024 * 1024),
                    space / (1024 * 1024)
                );
                return Err(eyre::eyre!("Insufficient disk space for model upload"));
            }
        }

        // Generate model path
        let model_path = models_dir.join(format!("{}.cvimodel", Self::generate_model_uid()));

        // Write model data with detailed error handling
        match std::fs::write(&model_path, model_data) {
            Ok(_) => {
                info!("Successfully saved model to {}", model_path.display());
                Ok(model_path)
            }
            Err(e) => {
                error!("Failed to write model to {}: {}", model_path.display(), e);
                Err(eyre::eyre!(
                    "Failed to save model file: {}. Check permissions and disk space",
                    e
                ))
            }
        }
    }

    async fn save_metadata(&self) -> Result<()> {
        let models_dir = PathBuf::from(MODELS_DIR);
        if let Err(e) = std::fs::create_dir_all(&models_dir) {
            error!("Failed to create models directory: {}", e);
            return Err(eyre::eyre!("Failed to create models directory: {}", e));
        }

        let metadata = self.available_models.read().await;
        let metadata_json = serde_json::to_string_pretty(&*metadata)
            .map_err(|e| eyre::eyre!("Failed to serialize metadata: {}", e))?;

        let temp_path = Path::new(METADATA_FILE).with_extension("tmp");

        // Write to temporary file first
        fs::write(&temp_path, metadata_json)
            .map_err(|e| eyre::eyre!("Failed to write metadata file: {}", e))?;

        // Atomically rename temporary file to actual file
        fs::rename(&temp_path, METADATA_FILE)
            .map_err(|e| eyre::eyre!("Failed to save metadata file: {}", e))?;

        debug!("Successfully saved metadata to {}", METADATA_FILE);
        Ok(())
    }

    async fn load_metadata(&self) -> Result<()> {
        if !Path::new(METADATA_FILE).exists() {
            debug!("No metadata file found at {}", METADATA_FILE);
            return Ok(());
        }

        let metadata_str = fs::read_to_string(METADATA_FILE)
            .map_err(|e| eyre::eyre!("Failed to read metadata file: {}", e))?;

        let metadata: HashMap<String, SerializableModelMetadata> =
            serde_json::from_str(&metadata_str)
                .map_err(|e| eyre::eyre!("Failed to parse metadata file: {}", e))?;

        let mut available = self.available_models.write().await;
        *available = metadata;

        debug!("Successfully loaded metadata from {}", METADATA_FILE);
        Ok(())
    }

    async fn load_model(&self, uid: &str, path: PathBuf) -> Result<()> {
        // Load model outside of lock
        let model = Model::new(path)?;

        // Minimal lock time for insertion
        let mut models = self.loaded_models.write().await;
        if models.contains_key(uid) {
            drop(model);
            return Err(eyre::eyre!("Model {} already loaded", uid));
        }
        models.insert(uid.to_string(), model);
        Ok(())
    }

    // Helper method to create ModelInfo from a loaded model
    fn create_model_info(
        &self,
        uid: &str,
        model: &CvitekModel,
        metadata: Option<&SerializableModelMetadata>,
    ) -> Result<ModelInfo> {
        let input_info = model.get_input_info()?;
        let output_info = model.get_output_info()?;

        let input_specs = input_info
            .into_iter()
            .map(|info| {
                let dims = info.shape.into_iter().map(|s| ProtoDimension {
                    size: s as u32,
                    name: info.name.clone(),
                    dynamic: false,
                }).collect();

                (info.name.clone(), Tensor {
                    values: Vec::new(),
                    shape: dims,
                })
            })
            .collect();

        let output_specs = output_info
            .into_iter()
            .map(|info| {
                let dims = info.shape.into_iter().map(|s| ProtoDimension {
                    size: s as u32,
                    name: info.name.clone(),
                    dynamic: false,
                }).collect();

                (info.name.clone(), Tensor {
                    values: Vec::new(),
                    shape: dims,
                })
            })
            .collect();

        Ok(ModelInfo {
            uid: uid.to_string(),
            metadata: Some(ModelMetadata::from(
                metadata.cloned().unwrap_or_default(),
            )),
            input_specs,
            output_specs,
            description: String::new(),
        })
    }
}

#[async_trait::async_trait]
impl Inference for ZBotInference {
    async fn upload_model(
        &self,
        model_data: Vec<u8>,
        metadata: Option<ModelMetadata>,
    ) -> Result<UploadModelResponse> {
        info!("Uploading new model");

        let model_path = self.save_model_binary(model_data).await?;
        let model_uid = model_path
            .file_stem()
            .and_then(|s| s.to_str())
            .ok_or_else(|| eyre::eyre!("Invalid model path"))?
            .to_string();

        self.register_model(model_uid.clone(), metadata.unwrap_or_default())
            .await?;

        self.load_models(vec![model_uid.clone()]).await?;

        Ok(UploadModelResponse {
            model_uid: model_uid,
            error: None,
        })
    }

    async fn load_models(&self, uids: Vec<String>) -> Result<LoadModelsResponse> {
        debug!("Loading models");

        // First check available models with read lock
        let available = self.available_models.read().await;
        let missing_uid = uids.iter().find(|uid| !available.contains_key(*uid));

        if let Some(uid) = missing_uid {
            return Ok(LoadModelsResponse {
                models: vec![],
                result: Some(ActionResponse {
                    success: false,
                    error: Some(Error {
                        code: ErrorCode::InvalidArgument as i32,
                        message: format!("Model {} not registered", uid),
                    }),
                }),
            });
        }
        drop(available);

        let mut failed_uids = Vec::new();
        let mut skipped_uids = Vec::new();

        {
            let loaded = self.loaded_models.read().await;
            skipped_uids.extend(uids.iter().filter(|uid| loaded.contains_key(*uid)).cloned());
        }

        for uid in uids.iter().filter(|uid| !skipped_uids.contains(*uid)) {
            let model_path = PathBuf::from(MODELS_DIR).join(format!("{}.cvimodel", uid));

            if !model_path.exists() {
                error!("Model file not found at {:?}", model_path);
                // Remove from available models since the file is missing
                let _model_metadata = self.available_models.write().await;
                // Save metadata after removal
                drop(_model_metadata);
                if let Err(e) = self.save_metadata().await {
                    error!(
                        "Failed to save metadata after removing missing model: {}",
                        e
                    );
                }
                failed_uids.push(uid.clone());
                continue;
            }

            match self.load_model(uid, model_path).await {
                Ok(_) => {
                    debug!("Loaded model {}", uid);
                }
                Err(e) => {
                    error!("Failed to load model {}: {}", uid, e);
                    failed_uids.push(uid.clone());
                }
            }
        }

        if !failed_uids.is_empty() {
            let mut message = format!("Failed to load models: {}", failed_uids.join(", "));
            if !skipped_uids.is_empty() {
                message.push_str(&format!(
                    ". Already loaded models: {}",
                    skipped_uids.join(", ")
                ));
            }
            return Ok(LoadModelsResponse {
                models: vec![],
                result: Some(ActionResponse {
                    success: false,
                    error: Some(Error {
                        code: ErrorCode::InvalidArgument as i32,
                        message,
                    }),
                }),
            });
        }

        let models = self.loaded_models.read().await;
        let available = self.available_models.read().await;

        // Build a list of ModelInfo objects, one for each loaded model
        let mut model_infos = Vec::new();
        for uid in models.keys() {
            let model = models.get(uid).unwrap();

            // Retrieve input_info
            let input_info = match model.get_input_info() {
                Ok(info) => info,
                Err(e) => {
                    error!("Failed to fetch input info for {}: {}", uid, e);
                    // Return an error, or skip this model. Here we choose to fail immediately:
                    return Err(eyre::eyre!("Failed to get input info for model {}: {}", uid, e));
                }
            };

            // Retrieve output_info
            let output_info = match model.get_output_info() {
                Ok(info) => info,
                Err(e) => {
                    error!("Failed to fetch output info for {}: {}", uid, e);
                    return Err(eyre::eyre!("Failed to get output info for model {}: {}", uid, e));
                }
            };

            // Convert input_info to input_specs
            let input_specs = input_info
                .into_iter()
                .map(|info| {
                    // Turn the shape[i32] into repeated Dimension
                    let dims = info.shape.into_iter().map(|dim_i32| {
                        ProtoDimension {
                            size: dim_i32 as u32,
                            name: String::new(),
                            dynamic: false,
                        }
                    }).collect();

                    (
                        info.name.clone(),
                        Tensor {
                            values: Vec::new(), // We only store shape for specs
                            shape: dims,
                        }
                    )
                })
                .collect::<std::collections::HashMap<_, _>>();

            // Convert output_info to output_specs
            let output_specs = output_info
                .into_iter()
                .map(|info| {
                    let dims = info.shape.into_iter().map(|dim_i32| {
                        ProtoDimension {
                            size: dim_i32 as u32,
                            name: String::new(),
                            dynamic: false,
                        }
                    }).collect();

                    (
                        info.name.clone(),
                        Tensor {
                            values: Vec::new(),
                            shape: dims,
                        }
                    )
                })
                .collect::<std::collections::HashMap<_, _>>();

            // Build the ModelInfo
            let metadata = Some(ModelMetadata::from(
                available.get(uid).cloned().unwrap_or_default(),
            ));
            model_infos.push(ModelInfo {
                uid: uid.clone(),
                metadata,
                input_specs,
                output_specs,
                description: String::new(),
            });
        }

        // Construct final response
        let response = LoadModelsResponse {
            models: model_infos,
            result: Some(ActionResponse {
                success: true,
                error: None,
            }),
        };

        Ok(response)
    }

    async fn unload_models(&self, uids: Vec<String>) -> Result<ActionResponse> {
        debug!("Unloading models");
        let mut models = self.loaded_models.write().await;
        let _model_metadata = self.available_models.write().await;

        for uid in uids {
            if let Some(model) = models.remove(&uid) {
                drop(model);
            } else {
                return Ok(ActionResponse {
                    success: false,
                    error: Some(Error {
                        code: ErrorCode::InvalidArgument as i32,
                        message: format!("Model {} not found", uid),
                    }),
                });
            }
        }

        Ok(ActionResponse {
            success: true,
            error: None,
        })
    }

    async fn get_models_info(
        &self,
        request: GetModelsInfoRequest,
    ) -> Result<GetModelsInfoResponse> {
        debug!("Getting models info");
        let metadata = self.available_models.read().await;
        let loaded_models = self.loaded_models.read().await;

        let model_infos = match request.filter {
            Some(Filter::ModelUids(ModelUids { uids })) => {
                if let Some(missing_uid) = uids.iter().find(|uid| !metadata.contains_key(*uid)) {
                    return Ok(GetModelsInfoResponse {
                        models: vec![],
                        error: Some(Error {
                            code: ErrorCode::InvalidArgument as i32,
                            message: format!("Model {} not found", missing_uid),
                        }),
                    });
                }

                let mut infos = Vec::new();
                for uid in uids {
                    if let Some(model) = loaded_models.get(&uid) {
                        infos.push(self.create_model_info(&uid, model, metadata.get(&uid))?);
                    }
                }
                infos
            }
            Some(Filter::All(_)) | None => {
                let mut infos = Vec::new();
                for (uid, _) in metadata.iter() {
                    if let Some(model) = loaded_models.get(uid) {
                        infos.push(self.create_model_info(uid, model, metadata.get(uid))?);
                    }
                }
                infos
            }
        };

        Ok(GetModelsInfoResponse {
            models: model_infos,
            error: None,
        })
    }

    async fn forward(
        &self,
        model_uid: String,
        inputs: HashMap<String, ProtoTensor>,
    ) -> Result<ForwardResponse> {
        let models = self.loaded_models.read().await;

        let model = match models.get(&model_uid) {
            Some(m) => m,
            None => {
                return Ok(ForwardResponse {
                    outputs: HashMap::new(),
                    error: Some(Error {
                        code: ErrorCode::InvalidArgument as i32,
                        message: format!("Model {} not found", model_uid),
                    }),
                });
            }
        };

        // Convert ProtoTensor to raw Vec<f32>
        let mut cvitek_inputs: HashMap<String, Vec<f32>> = HashMap::new();
        for (name, tensor) in &inputs {
            cvitek_inputs.insert(name.clone(), tensor.values.clone());
        }

        let inference_result = match model.infer(cvitek_inputs) {
            Ok(outputs) => outputs,
            Err(e) => {
                error!("Inference failed: {:?}", e);
                return Ok(ForwardResponse {
                    outputs: HashMap::new(),
                    error: Some(Error {
                        code: ErrorCode::HardwareFailure as i32,
                        message: format!("Inference failed: {:?}", e),
                    }),
                });
            }
        };

        let output_meta = match model.get_output_info() {
            Ok(info) => info,
            Err(e) => {
                error!("Failed to retrieve output tensor info: {}", e);
                return Ok(ForwardResponse {
                    outputs: HashMap::new(),
                    error: Some(Error {
                        code: ErrorCode::HardwareFailure as i32,
                        message: format!("Failed to retrieve output metadata: {}", e),
                    }),
                });
            }
        };

        let mut output_info_map = HashMap::new();
        for ti in output_meta {
            output_info_map.insert(ti.name.clone(), ti);
        }

        let mut final_outputs = HashMap::new();
        for (name, values) in inference_result {
            let shape = if let Some(ti) = output_info_map.get(&name) {
                ti.shape.iter().map(|&s| ProtoDimension {
                    size: s as u32,
                    name: String::new(),
                    dynamic: false,
                }).collect()
            } else {
                Vec::new()
            };

            final_outputs.insert(name, Tensor { values, shape });
        }

        Ok(ForwardResponse {
            outputs: final_outputs,
            error: None,
        })
    }
}
