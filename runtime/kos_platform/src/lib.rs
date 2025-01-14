mod actuator;
mod firmware;
mod imu;

pub use actuator::*;
pub use firmware::*;
pub use imu::*;

use kos::kos_proto::actuator::actuator_service_server::ActuatorServiceServer;
use kos::kos_proto::imu::imu_service_server::ImuServiceServer;
use kos::services::{ActuatorServiceImpl, IMUServiceImpl, OperationsServiceImpl};
use kos::{Platform, ServiceEnum};
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
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

            Ok(services)
        })
    }

    fn shutdown(&mut self) -> eyre::Result<()> {
        Ok(())
    }
}
