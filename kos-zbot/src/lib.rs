mod actuator;
mod firmware;
mod imu_bmi088;
mod imu_bno055;
mod led_matrix;
mod model;

pub use actuator::*;
pub use firmware::*;
pub use led_matrix::*;
pub use model::*;

use crate::imu_bmi088::ZBotBMI088;
use crate::imu_bno055::ZBotBNO055;
use kos::{
    hal::IMU,
    kos_proto::actuator::actuator_service_server::ActuatorServiceServer,
    kos_proto::imu::imu_service_server::ImuServiceServer,
    kos_proto::inference::inference_service_server::InferenceServiceServer,
    kos_proto::led_matrix::led_matrix_service_server::LedMatrixServiceServer,
    services::{
        ActuatorServiceImpl, IMUServiceImpl, InferenceServiceImpl, LEDMatrixServiceImpl,
        OperationsServiceImpl,
    },
    Platform, ServiceEnum,
};
use std::{future::Future, pin::Pin, sync::Arc};
use tonic::async_trait;
use tracing::{error, info, warn};

pub struct ZBotPlatform {}

impl ZBotPlatform {
    pub fn new() -> Self {
        Self {}
    }
}

impl Default for ZBotPlatform {
    fn default() -> Self {
        ZBotPlatform::new()
    }
}

#[async_trait]
impl Platform for ZBotPlatform {
    fn name(&self) -> &'static str {
        "ZBot"
    }

    fn serial(&self) -> String {
        "00000000".to_string()
    }

    fn initialize(&mut self, _operations_service: Arc<OperationsServiceImpl>) -> eyre::Result<()> {
        Ok(())
    }

    fn create_services<'a>(
        &'a self,
        _operations_service: Arc<OperationsServiceImpl>,
    ) -> Pin<Box<dyn Future<Output = eyre::Result<Vec<ServiceEnum>>> + Send + 'a>> {
        Box::pin(async move {
            let actuator_list = [
                11, 12, 13, 14, 21, 22, 23, 24, 31, 32, 33, 34, 35, 36, 41, 42, 43, 44, 45, 46
            ];

            let actuator = ZBotActuator::new(actuator_list.as_slice()).await?;

            let mut services = vec![ServiceEnum::Actuator(ActuatorServiceServer::new(
                ActuatorServiceImpl::new(Arc::new(actuator)),
            ))];

            // Try BNO055 first, fall back to BMI088.
            // If both fail, we log the error and continue without the IMU service.
            let imu_service = match ZBotBNO055::new("/dev/i2c-1") {
                Ok(bno055) => {
                    info!("Successfully initialized BNO055");
                    Some(Arc::new(bno055) as Arc<dyn IMU>)
                }
                Err(e) => {
                    warn!("BNO055 initialization failed ({}), attempting BMI088", e);
                    match ZBotBMI088::new("/dev/i2c-1") {
                        Ok(bmi088) => {
                            info!("Successfully initialized BMI088");
                            Some(Arc::new(bmi088) as Arc<dyn IMU>)
                        }
                        Err(e) => {
                            error!("Failed to initialize BMI088: {}", e);
                            None
                        }
                    }
                }
            };

            if let Some(imu) = imu_service {
                services.push(ServiceEnum::Imu(ImuServiceServer::new(
                    IMUServiceImpl::new(imu),
                )));
            } else {
                error!("Failed to initialize both BNO055 and BMI088 IMUs. Continuing without IMU sensor.");
            }

            match ZBotInference::new() {
                Ok(inference) => {
                    services.push(ServiceEnum::Inference(InferenceServiceServer::new(
                        InferenceServiceImpl::new(Arc::new(inference)),
                    )));
                }
                Err(e) => {
                    error!("Failed to initialize Inference: {}", e);
                }
            }

            match ZBotLEDMatrix::new("/dev/i2c-1") {
                Ok(led_matrix) => {
                    services.push(ServiceEnum::LEDMatrix(LedMatrixServiceServer::new(
                        LEDMatrixServiceImpl::new(Arc::new(led_matrix)),
                    )));
                }
                Err(e) => {
                    error!("Failed to initialize LEDMatrix: {}", e);
                }
            }

            Ok(services)
        })
    }

    fn shutdown(&mut self) -> eyre::Result<()> {
        Ok(())
    }
}
