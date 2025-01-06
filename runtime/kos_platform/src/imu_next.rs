use async_trait::async_trait;
use bno055::{Bno055, BusType};
use eyre::Result;
use kos_core::{
    google_proto::longrunning::Operation,
    hal::{EulerAnglesResponse, ImuValuesResponse, QuaternionResponse, IMU},
    kos_proto::common::{ActionResponse, Error, ErrorCode},
};
use std::{sync::Arc, time::Duration};
use tokio::sync::Mutex;
use tracing::{debug, error, info};

pub struct ZBotIMU {
    imu: Arc<Mutex<Bno055<BusType>>>,
}

impl ZBotIMU {
    pub fn new(i2c_bus: &str) -> Result<Self> {
        info!("Initializing ZerothIMU with I2C bus: {}", i2c_bus);
        
        let mut imu = Bno055::new(i2c_bus)?;
        
        // Initialize the IMU
        imu.init()?;

        // Set to NDOF mode (Nine Degrees of Freedom)
        imu.set_mode(bno055::Mode::Ndof)?;
        
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
        let imu = self.imu.lock().await;
        
        let accel = imu.accel_data()?;
        let gyro = imu.gyro_data()?;
        let mag = imu.mag_data()?;

        Ok(ImuValuesResponse {
            accel_x: accel.x as f64,
            accel_y: accel.y as f64,
            accel_z: accel.z as f64,
            gyro_x: gyro.x as f64,
            gyro_y: gyro.y as f64,
            gyro_z: gyro.z as f64,
            mag_x: Some(mag.x as f64),
            mag_y: Some(mag.y as f64),
            mag_z: Some(mag.z as f64),
            error: None,
        })
    }

    async fn calibrate(&self) -> Result<Operation> {
        info!("Starting IMU calibration");
        let mut imu = self.imu.lock().await;
        
        // Get current calibration status
        let status = imu.get_calibration_status()?;
        
        if status.sys == 3 {
            debug!("IMU already fully calibrated");
            return Ok(Operation {
                name: "operations/calibrate_imu/0".to_string(),
                metadata: None,
                done: true,
                result: None,
            });
        }

        // The BNO055 self-calibrates during normal operation
        // We'll just return an operation indicating it's in progress
        Ok(Operation {
            name: "operations/calibrate_imu/0".to_string(),
            metadata: None,
            done: false,
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
        
        // Attempt to reset the system
        match imu.reset_system() {
            Ok(_) => {
                // Re-initialize after reset
                if let Err(e) = imu.init() {
                    error!("Failed to reinitialize IMU after reset: {}", e);
                    return Ok(ActionResponse {
                        success: false,
                        error: Some(Error {
                            code: ErrorCode::HardwareFailure as i32,
                            message: format!("Failed to reinitialize IMU: {}", e),
                        }),
                    });
                }
                
                // Set back to NDOF mode
                if let Err(e) = imu.set_mode(bno055::Mode::Ndof) {
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

    async fn get_euler(&self) -> Result<EulerAnglesResponse> {
        let imu = self.imu.lock().await;
        let euler = imu.euler_angles()?;
        
        Ok(EulerAnglesResponse {
            roll: euler.roll as f64,
            pitch: euler.pitch as f64,
            yaw: euler.heading as f64,
            error: None,
        })
    }

    async fn get_quaternion(&self) -> Result<QuaternionResponse> {
        let imu = self.imu.lock().await;
        let quat = imu.quaternion()?;
        
        Ok(QuaternionResponse {
            w: quat.w as f64,
            x: quat.x as f64,
            y: quat.y as f64,
            z: quat.z as f64,
            error: None,
        })
    }
}
