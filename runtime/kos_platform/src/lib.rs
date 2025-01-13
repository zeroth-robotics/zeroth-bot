mod actuator;
mod firmware;
mod imu;
mod inference; 

pub use actuator::*;
pub use firmware::*;
pub use imu::*;
pub use inference::*;
use kos_core::kos_proto::actuator::actuator_service_server::ActuatorServiceServer;
use kos_core::kos_proto::imu::imu_service_server::ImuServiceServer;
use kos_core::services::{ActuatorServiceImpl, IMUServiceImpl, OperationsServiceImpl};
use kos_core::{Platform, ServiceEnum};
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;
use tonic::async_trait;

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
            let actuator = ZBotActuator::new().await?;

            let mut services = vec![ServiceEnum::Actuator(ActuatorServiceServer::new(
                ActuatorServiceImpl::new(Arc::new(actuator)),
            ))];

            match ZBotIMU::new("/dev/i2c-1") {
                Ok(imu) => {
                    services.push(ServiceEnum::Imu(ImuServiceServer::new(
                        IMUServiceImpl::new(Arc::new(imu)),
                    )));
                }
                Err(e) => {
                    eprintln!("Failed to initialize IMU: {}", e);
                }
            }

            let inference = ZBotInference::new(imu, actuator);
            services.push(ServiceEnum::Inference(InferenceServiceServer::new(
                InferenceServiceImpl::new(Arc::new(inference)),
            )));

            Ok(services)
        })
    }

    fn shutdown(&mut self) -> eyre::Result<()> {
        Ok(())
    }
}
