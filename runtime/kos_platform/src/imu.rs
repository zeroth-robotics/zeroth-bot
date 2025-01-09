use async_trait::async_trait;

use eyre::Result;
use kos_core::{
    google_proto::longrunning::Operation,
    hal::{EulerAnglesResponse, ImuAdvancedValuesResponse, ImuValuesResponse, QuaternionResponse, IMU},
    kos_proto::common::{ActionResponse, Error, ErrorCode},
};
use linux_bno055::{Bno055, OperationMode};
use std::{sync::Arc, time::Duration};
use tokio::sync::Mutex;
use tracing::{debug, error, info};

pub struct ZBotIMU {
    imu: Arc<Mutex<Bno055>>,
}

impl ZBotIMU {
    pub fn new(i2c_bus: &str) -> Result<Self> {
        info!("Initializing ZerothIMU with I2C bus: {}", i2c_bus);

        let imu = Bno055::new(i2c_bus)?;

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

        let accel = imu.get_accelerometer()?;
        let gyro = imu.get_gyroscope()?;
        let mag = imu.get_magnetometer().ok();

        Ok(ImuValuesResponse {
            accel_x: accel.x as f64,
            accel_y: accel.y as f64,
            accel_z: accel.z as f64,
            gyro_x: gyro.x as f64,
            gyro_y: gyro.y as f64,
            gyro_z: gyro.z as f64,
            mag_x: mag.as_ref().map(|m| m.x as f64),
            mag_y: mag.as_ref().map(|m| m.y as f64),
            mag_z: mag.as_ref().map(|m| m.z as f64),
            error: None,
        })
    }

    async fn get_advanced_values(&self) -> Result<ImuAdvancedValuesResponse> {
        let mut imu = self.imu.lock().await;

        let lin_acc = imu.get_linear_acceleration().ok();
        let grav = imu.get_gravity_vector().ok();

        Ok(ImuAdvancedValuesResponse {
            lin_acc_x: lin_acc.as_ref().map(|g| g.x as f64),
            lin_acc_y: lin_acc.as_ref().map(|g| g.y as f64),
            lin_acc_z: lin_acc.as_ref().map(|g| g.z as f64),
            grav_x: grav.as_ref().map(|g| g.x as f64),
            grav_y: grav.as_ref().map(|g| g.y as f64),
            grav_z: grav.as_ref().map(|g| g.z as f64),
            temp: imu.get_temperature().ok().map(|t| t as f64),
            error: None,
        })
    }

    async fn get_euler(&self) -> Result<EulerAnglesResponse> {
        let mut imu = self.imu.lock().await;
        let euler = imu.get_euler_angles()?;

        Ok(EulerAnglesResponse {
            roll: euler.roll as f64,
            pitch: euler.pitch as f64,
            yaw: euler.yaw as f64,
            error: None,
        })
    }

    async fn get_quaternion(&self) -> Result<QuaternionResponse> {
        let mut imu = self.imu.lock().await;
        let quat = imu.get_quaternion()?;

        Ok(QuaternionResponse {
            w: quat.w as f64,
            x: quat.x as f64,
            y: quat.y as f64,
            z: quat.z as f64,
            error: None,
        })
    }

    async fn calibrate(&self) -> Result<Operation> {
        info!("Starting IMU calibration");

        let mut imu = self.imu.lock().await;

        imu.reset()?;
        let cal_status = imu.get_calibration_status()?;

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
        let mut imu = self.imu.lock().await;

        match imu.reset() {
            Ok(_) => {
                // Reset successful, now set mode back to NDOF
                if let Err(e) = imu.set_mode(OperationMode::Ndof) {
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
