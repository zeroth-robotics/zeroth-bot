use async_trait::async_trait;

use eyre::Result;
use kos_core::{
    google_proto::longrunning::Operation,
    hal::{EulerAnglesResponse, ImuValuesResponse, QuaternionResponse, IMU},
    kos_proto::common::{ActionResponse, Error, ErrorCode},
};
use std::{sync::Arc, time::Duration};
use tokio::sync::Mutex;
use tracing::{debug, error, info};
use crate::firmware::QMI8658;

pub struct ZBotIMU {
    imu: Arc<Mutex<QMI8658>>,
}

impl ZBotIMU {
    pub fn new(i2c_bus: &str) -> Result<Self> {
        info!("Initializing ZerothIMU with I2C bus: {}", i2c_bus);
        
        let mut imu = QMI8658::new(i2c_bus)?;
        
        // Initialize the IMU
        imu.init()?;

        Ok(Self {
            imu: Arc::new(Mutex::new(imu)),
        })
    }
}

impl Default for ZBotIMU {
    fn default() -> Self {
        unimplemented!("ZBotIMU cannot be default, it requires I2C bus configuration")
    }
}

#[async_trait]
impl IMU for ZBotIMU {
    async fn get_values(&self) -> Result<ImuValuesResponse> {
        let mut imu = self.imu.lock().await;
        
        let data = imu.read_data()
            .map_err(|e| eyre::eyre!("Failed to read QMI8658 data: {}", e))?;

        Ok(ImuValuesResponse {
            accel_x: data.acc_x as f64,
            accel_y: data.acc_y as f64,
            accel_z: data.acc_z as f64,
            gyro_x: data.gyro_x as f64,
            gyro_y: data.gyro_y as f64,
            gyro_z: data.gyro_z as f64,
            mag_x: None,
            mag_y: None,
            mag_z: None,
            error: None,
        })
    }

    async fn calibrate(&self) -> Result<Operation> {
        unimplemented!("Calibration is not supported for QMI8658")
    }

    async fn zero(
        &self,
        duration: Option<Duration>,
        max_retries: Option<u32>,
        max_angular_error: Option<f32>,
        max_vel: Option<f32>,
        max_accel: Option<f32>,
    ) -> Result<ActionResponse> {
        unimplemented!("Zeroing is not supported for QMI8658")
    }

    async fn get_euler(&self) -> Result<EulerAnglesResponse> {
        unimplemented!("Euler angles are not supported for QMI8658")
    }

    async fn get_quaternion(&self) -> Result<QuaternionResponse> {
        unimplemented!("Quaternions are not supported for QMI8658")
    }
}
