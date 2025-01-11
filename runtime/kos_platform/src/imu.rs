use async_trait::async_trait;

use eyre::Result;
use kos_core::{
    google_proto::longrunning::Operation,
    hal::{EulerAnglesResponse, ImuAdvancedValuesResponse, ImuValuesResponse, QuaternionResponse, IMU},
    kos_proto::common::{ActionResponse, Error, ErrorCode},
};
use linux_bno055::{Bno055Reader, OperationMode};
use std::{sync::Arc, time::Duration};
use tokio::sync::Mutex;
use tracing::{debug, error, info};

pub struct ZBotIMU {
    imu: Arc<Bno055Reader>,
}

impl ZBotIMU {
    pub fn new(i2c_bus: &str) -> Result<Self> {
        info!("Initializing ZerothIMU with I2C bus: {}", i2c_bus);

        let imu = Bno055Reader::new(i2c_bus)?;

        Ok(Self {
            imu: Arc::new(imu),
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
        let data = self.imu.get_data()?;

        Ok(ImuValuesResponse {
            accel_x: data.accelerometer.x as f64,
            accel_y: data.accelerometer.y as f64,
            accel_z: data.accelerometer.z as f64,
            gyro_x: data.gyroscope.x as f64,
            gyro_y: data.gyroscope.y as f64,
            gyro_z: data.gyroscope.z as f64,
            mag_x: Some(data.magnetometer.x as f64),
            mag_y: Some(data.magnetometer.y as f64),
            mag_z: Some(data.magnetometer.z as f64),
            error: None,
        })
    }

    async fn get_advanced_values(&self) -> Result<ImuAdvancedValuesResponse> {
        let data = self.imu.get_data()?;

        Ok(ImuAdvancedValuesResponse {
            lin_acc_x: Some(data.linear_acceleration.x as f64),
            lin_acc_y: Some(data.linear_acceleration.y as f64),
            lin_acc_z: Some(data.linear_acceleration.z as f64),
            grav_x: Some(data.gravity.x as f64),
            grav_y: Some(data.gravity.y as f64),
            grav_z: Some(data.gravity.z as f64),
            temp: Some(data.temperature as f64),
            error: None,
        })
    }

    async fn get_euler(&self) -> Result<EulerAnglesResponse> {
        let data = self.imu.get_data()?;

        Ok(EulerAnglesResponse {
            roll: data.euler.roll as f64,
            pitch: data.euler.pitch as f64,
            yaw: data.euler.yaw as f64,
            error: None,
        })
    }

    async fn get_quaternion(&self) -> Result<QuaternionResponse> {
        let data = self.imu.get_data()?;

        Ok(QuaternionResponse {
            w: data.quaternion.w as f64,
            x: data.quaternion.x as f64,
            y: data.quaternion.y as f64,
            z: data.quaternion.z as f64,
            error: None,
        })
    }

    async fn calibrate(&self) -> Result<Operation> {
        info!("Starting IMU calibration");

        self.imu.reset()?;
        let cal_status = self.imu.get_data()?.calibration_status;

        debug!(
            "Calibration Status - Sys: {}, Gyro: {}, Accel: {}, Mag: {}",
            (cal_status >> 6) & 0x03,
            (cal_status >> 4) & 0x03,
            (cal_status >> 2) & 0x03,
            cal_status & 0x03
        );

        Ok(Operation {
            name: "operations/calibrate_imu/0".to_string(),
            metadata: None,
            done: true,
            result: None,
        })
    }

    async fn zero(
        &self,
        duration: Option<Duration>,
        max_retries: Option<u32>,
        max_angular_error: Option<f32>,
        max_vel: Option<f32>,
        max_accel: Option<f32>,
    ) -> Result<ActionResponse> {
        match self.imu.reset() {
            Ok(_) => {
                // Reset successful, now set mode back to NDOF
                if let Err(e) = self.imu.set_mode(OperationMode::Ndof) {
                    error!("Failed to set IMU mode after reset: {}", e);
                    return Ok(ActionResponse {
                        success: false,
                        error: Some(Error {
                            code: ErrorCode::HardwareFailure as i32,
                            message: format!("Failed to set IMU mode: {}", e),
                        }),
                    });
                }

                Ok(ActionResponse {
                    success: true,
                    error: None,
                })
            }
            Err(e) => {
                error!("Failed to zero IMU: {}", e);
                Ok(ActionResponse {
                    success: false,
                    error: Some(Error {
                        code: ErrorCode::HardwareFailure as i32,
                        message: format!("Failed to zero IMU: {}", e),
                    }),
                })
            }
        }
    }
}
