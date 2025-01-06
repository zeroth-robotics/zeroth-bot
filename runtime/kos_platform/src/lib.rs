mod actuator;
mod firmware;
mod imu;

pub use actuator::*;
pub use firmware::*;
pub use imu::*;

use kos_core::{Platform, ServiceEnum};
use tonic::async_trait;
use std::sync::Arc;
use kos_core::services::{ActuatorServiceImpl, IMUServiceImpl, OperationsServiceImpl};
use kos_core::kos_proto::actuator::actuator_service_server::ActuatorServiceServer;
use kos_core::kos_proto::imu::imu_service_server::ImuServiceServer;
use std::future::Future;
use std::pin::Pin;
use std::time::Duration;

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
            let imu = ZBotIMU::new("/dev/i2c-1")?;

            Ok(vec![
                ServiceEnum::Actuator(ActuatorServiceServer::new(
                    ActuatorServiceImpl::new(Arc::new(actuator))
                )),
                ServiceEnum::Imu(ImuServiceServer::new(
                    IMUServiceImpl::new(Arc::new(imu))
                ))
            ])
        })
    }

    fn shutdown(&mut self) -> eyre::Result<()> {
        Ok(())
    }
}
