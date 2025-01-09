use crate::Model;
use eyre::Result;
use kos_core::hal::Inference;
use kos_core::kos_proto::inference::*;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{debug, error, info};
use uuid::Uuid;
use serde_json::Value;
use std::fs;
use std::io::{self, Write};
use std::path::Path;

const MODELS_DIR: &'static str = "/opt/models";
const METADATA_FILE: &'static str = "/opt/models/metadata.json";

pub struct ZBotInference {
    loaded_models: Arc<RwLock<HashMap<String, Model>>>,
    available_models: Arc<RwLock<HashMap<String, ModelMetadata>>>,
}

impl ZBotInference {
    pub fn new() -> Self {
        let inference = Self {
            loaded_models: Arc::new(RwLock::new(HashMap::new())),
            available_models: Arc::new(RwLock::new(HashMap::new())),
        };

        // Load metadata in a blocking task to avoid async in new()
        let inference_clone = inference.clone();
        tokio::spawn(async move {
            if let Err(e) = inference_clone.load_metadata().await {
                error!("Failed to load metadata: {}", e);
            }
        });

        inference
    }

    fn generate_model_uid() -> String {
        Uuid::new_v4().to_string()
    }

    async fn register_model(&self, model_uid: String, metadata: ModelMetadata) -> Result<()> {
        let mut all_models = self.available_models.write().await;
        all_models.insert(model_uid.clone(), metadata);
        drop(all_models); // Release the write lock before saving
        
        self.save_metadata().await?;
        Ok(())
    }

    async fn save_model_binary(&self, model_data: Vec<u8>) -> Result<PathBuf> {
        let models_dir = PathBuf::from(MODELS_DIR);
        
        // Create models directory with detailed error handling
        if let Err(e) = std::fs::create_dir_all(&models_dir) {
            error!("Failed to create models directory at {}: {}", models_dir.display(), e);
            return Err(eyre::eyre!(
                "Failed to create models directory: {}. Check permissions and disk space", e
            ));
        }
        
        // Check available disk space (require at least model size + 10MB buffer)
        let required_space = model_data.len() as u64 + 10 * 1024 * 1024; // model size + 10MB
        if let Ok(space) = fs2::available_space(&models_dir) {
            if space < required_space {
                error!("Insufficient disk space. Required: {}MB, Available: {}MB", 
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
                    "Failed to save model file: {}. Check permissions and disk space", e
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

        let temp_path = Path::new(&Self::METADATA_FILE).with_extension("tmp");
        
        // Write to temporary file first
        fs::write(&temp_path, metadata_json)
            .map_err(|e| eyre::eyre!("Failed to write metadata file: {}", e))?;
        
        // Atomically rename temporary file to actual file
        fs::rename(&temp_path, Self::METADATA_FILE)
            .map_err(|e| eyre::eyre!("Failed to save metadata file: {}", e))?;

        debug!("Successfully saved metadata to {}", Self::METADATA_FILE);
        Ok(())
    }

    async fn load_metadata(&self) -> Result<()> {
        if !Path::new(Self::METADATA_FILE).exists() {
            debug!("No metadata file found at {}", Self::METADATA_FILE);
            return Ok(());
        }

        let metadata_str = fs::read_to_string(Self::METADATA_FILE)
            .map_err(|e| eyre::eyre!("Failed to read metadata file: {}", e))?;

        let metadata: HashMap<String, ModelMetadata> = serde_json::from_str(&metadata_str)
            .map_err(|e| eyre::eyre!("Failed to parse metadata file: {}", e))?;

        let mut available = self.available_models.write().await;
        *available = metadata;

        debug!("Successfully loaded metadata from {}", Self::METADATA_FILE);
        Ok(())
    }

    async fn load_model(&self, uid: &str, path: PathBuf) -> Result<Model> {
        // Load model outside of lock
        let model = Model::new(path)?;
        
        // Minimal lock time for insertion
        let mut models = self.loaded_models.write().await;
        if models.contains_key(uid) {
            drop(model);
            return Err(eyre::eyre!("Model {} already loaded", uid));
        }
        models.insert(uid.to_string(), model);
        Ok(model)
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

        self.register_model(model_uid, metadata.unwrap_or_default()).await?;
        
        self.load_models(vec![model_uid]).await?;
        
        Ok(UploadModelResponse {
            model_uid,
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
                result: ActionResponse {
                    success: false,
                    error: Some(kos_core::kos_proto::common::Error {
                        code: kos_core::kos_proto::common::ErrorCode::NotFound as i32,
                        message: format!("Model {} not registered", uid),
                    }),
                },
            });
        }

        drop(available);
        
        let mut failed_uids = Vec::new();

        for uid in uids {
            let model_path = PathBuf::from(MODELS_DIR).join(format!("{}.cvimodel", uid));
            
            if !model_path.exists() {
                error!("Model file not found at {:?}", model_path);
                // Remove from available models since the file is missing
                let mut available = self.available_models.write().await;
                available.remove(&uid);
                // Save metadata after removal
                drop(available);
                if let Err(e) = self.save_metadata().await {
                    error!("Failed to save metadata after removing missing model: {}", e);
                }
                failed_uids.push(uid);
                continue;
            }

            match self.load_model(uid, model_path).await {
                Ok(model) => {
                    debug!("Loaded model {}", uid);
                }
                Err(e) => {
                    error!("Failed to load model {}: {}", uid, e);
                    failed_uids.push(uid);
                }
            }
        }

        if !failed_uids.is_empty() {
            return Ok(LoadModelsResponse {
                success: false,
                error: Some(Error {
                    code: ErrorCode::NotFound as i32,
                    message: format!("Failed to load models: {}", failed_uids.join(", ")),
                }),
            });
        }

        let models = self.loaded_models.read().await;
        
        Ok(LoadModelsResponse {
            models: models
                .keys()
                .map(|uid| ModelInfo {
                    uid: uid.clone(),
                    metadata: available.get(uid).cloned().unwrap_or_default(),
                })
                .collect(),
            result: ActionResponse {
                success: true,
                error: None,
            },
        })
    }

    async fn unload_models(&self, uids: Vec<String>) -> Result<ActionResponse> {
        debug!("Unloading models");
        let mut models = self.loaded_models.write().await;
        let mut model_metadata = self.available_models.write().await;

        for uid in uids {
            if let Some(model) = models.remove(&uid) {
                drop(model);
            } else {
                return Ok(ActionResponse {
                    success: false,
                    error: Some(kos_core::kos_proto::common::Error {
                        code: kos_core::kos_proto::common::ErrorCode::NotFound as i32,
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

    async fn get_models_info(&self, request: GetModelsInfoRequest) -> Result<GetModelsInfoResponse> {
        debug!("Getting models info");
        let metadata = self.available_models.read().await;
        
        let model_infos = match request.filter {
            Some(filter::Filter::ModelUids(ModelUids { uids })) => {
                // First check if all requested models exist
                if let Some(missing_uid) = uids.iter().find(|uid| !metadata.contains_key(*uid)) {
                    return Ok(GetModelsInfoResponse {
                        models: vec![],
                        error: Some(kos_core::kos_proto::common::Error {
                            code: kos_core::kos_proto::common::ErrorCode::NotFound as i32,
                            message: format!("Model {} not found", missing_uid),
                        }),
                    });
                }

                // If all exist, map them to ModelInfo
                uids.iter()
                    .map(|uid| ModelInfo {
                        uid: uid.clone(),
                        metadata: metadata.get(uid).cloned().unwrap_or_default(),
                    })
                    .collect()
            }
            Some(filter::Filter::All(_)) | None => {
                metadata
                    .keys()
                    .map(|uid| ModelInfo {
                        uid: uid.clone(),
                        metadata: metadata.get(uid).cloned().unwrap_or_default(),
                    })
                    .collect()
            }
        };

        Ok(GetModelsInfoResponse {
            models: model_infos,
            error: None,
        })
    }

    async fn forward(&self, model_uid: String, inputs: Vec<f32>) -> Result<ForwardResponse> {
        let models = self.loaded_models.read().await;
        
        if let Some(model) = models.get(&model_uid) {
            match model.infer(&inputs) {
                Ok(outputs) => Ok(ForwardResponse {
                    outputs,
                    error: None,
                }),
                Err(e) => {
                    error!("Inference failed: {}", e);
                    Ok(ForwardResponse {
                        outputs: vec![],
                        error: Some(kos_core::kos_proto::common::Error {
                            code: kos_core::kos_proto::common::ErrorCode::HardwareFailure as i32,
                            message: format!("Inference failed: {}", e),
                        }),
                    })
                }
            }
        } else {
            Ok(ForwardResponse {
                outputs: vec![],
                error: Some(kos_core::kos_proto::common::Error {
                    code: kos_core::kos_proto::common::ErrorCode::NotFound as i32,
                    message: format!("Model {} not found", model_uid),
                }),
            })
        }
    }
}
