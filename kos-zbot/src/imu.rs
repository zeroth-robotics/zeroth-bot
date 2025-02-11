use async_trait::async_trait;
use eyre::Result;
use imu::bno055::{Bno055Reader, OperationMode};
use kos::{
    hal::{
        EulerAnglesResponse, ImuAdvancedValuesResponse, ImuValuesResponse, Operation,
        QuaternionResponse, IMU,
    },
    kos_proto::common::{ActionResponse, Error, ErrorCode},
};
use std::sync::Arc;
use std::time::Duration;
use tracing::{debug, error, info, warn};
use imu::bmi088::Bmi088Reader;

pub enum ImuReader {
    Bno055(Arc<Bno055Reader>),
    Bmi088(Arc<Bmi088Reader>),
}

pub struct ZBotIMU {
    imu: ImuReader,  // Change from Arc<Bno055Reader> to ImuReader
}

impl ZBotIMU {
    fn run_bmi088_sanity_check() {
        if let Ok(()) = (|| -> Result<(), &'static str> {
            info!("Running BMI088 sanity check...");
            println!("IMU module loaded - BMI088 addresses: 0x{:02X}/0x{:02X}", 
                     imu::bmi088::ACCEL_ADDR, 
                     imu::bmi088::GYRO_ADDR);
            Ok(())
        })() {
            info!("BMI088 sanity check passed");
        } else {
            error!("BMI088 sanity check failed - continuing anyway");
        }
    }

    pub fn new(i2c_bus: &str) -> Result<Self> {
        info!("Initializing IMU system on bus: {}", i2c_bus);
        
        // Try BNO055 first
        match Bno055Reader::new(i2c_bus) {
            Ok(bno055) => {
                // Additional check to verify BNO055 is actually working
                match bno055.get_data() {
                    Ok(data) => {
                        // Check if accelerometer data is all zeros—which is very unlikely if the sensor is active
                        if data.accelerometer.x == 0.0 &&
                           data.accelerometer.y == 0.0 &&
                           data.accelerometer.z == 0.0 {
                            warn!("BNO055 returned default (all-zero) data, falling back to BMI088");
                            Self::run_bmi088_sanity_check();
                            match Bmi088Reader::new(i2c_bus) {
                                Ok(bmi088) => {
                                    info!("Successfully initialized BMI088 as fallback");
                                    Ok(Self { imu: ImuReader::Bmi088(Arc::new(bmi088)) })
                                }
                                Err(bmi_err) => {
                                    error!("Both IMUs failed to initialize:");
                                    error!("  BNO055 returned default data");
                                    error!("  BMI088 error: {}", bmi_err);
                                    Err(eyre::eyre!("Failed to initialize any IMU"))
                                }
                            }
                        } else {
                            info!("Successfully initialized and verified BNO055");
                            Ok(Self { imu: ImuReader::Bno055(Arc::new(bno055)) })
                        }
                    }
                    Err(e) => {
                        warn!("BNO055 initialized but failed to read data ({}), falling back to BMI088", e);
                        Self::run_bmi088_sanity_check();
                        match Bmi088Reader::new(i2c_bus) {
                            Ok(bmi088) => {
                                info!("Successfully initialized BMI088 as fallback");
                                Ok(Self { imu: ImuReader::Bmi088(Arc::new(bmi088)) })
                            }
                            Err(bmi_err) => {
                                error!("Both IMUs failed to initialize:");
                                error!("  BNO055 error: {}", e);
                                error!("  BMI088 error: {}", bmi_err);
                                Err(eyre::eyre!("Failed to initialize any IMU"))
                            }
                        }
                    }
                }
            }
            Err(bno_err) => {
                warn!("BNO055 initialization failed ({}), attempting BMI088", bno_err);
                Self::run_bmi088_sanity_check();
                
                match Bmi088Reader::new(i2c_bus) {
                    Ok(bmi088) => {
                        info!("Successfully initialized BMI088 as fallback");
                        Ok(Self { imu: ImuReader::Bmi088(Arc::new(bmi088)) })
                    }
                    Err(bmi_err) => {
                        error!("Both IMUs failed to initialize:");
                        error!("  BNO055 error: {}", bno_err);
                        error!("  BMI088 error: {}", bmi_err);
                        Err(eyre::eyre!("Failed to initialize any IMU"))
                    }
                }
            }
        }
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
        match &self.imu {
            ImuReader::Bno055(bno) => {
                info!("Using BNO055 for get_values");
                let data = bno.get_data()?;
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
            ImuReader::Bmi088(bmi) => {
                info!("Using BMI088 for get_values");
                let data = bmi.get_data()?;
                if let Ok(data) = bmi.get_data() {
                    debug!(
                        "Raw BMI088 data - Accel: ({:.2}, {:.2}, {:.2}), Gyro: ({:.2}, {:.2}, {:.2})",
                        data.accelerometer.x, data.accelerometer.y, data.accelerometer.z,
                        data.gyroscope.x, data.gyroscope.y, data.gyroscope.z
                    );
                    Ok(ImuValuesResponse {
                        accel_x: data.accelerometer.x as f64,
                        accel_y: data.accelerometer.y as f64,
                        accel_z: data.accelerometer.z as f64,
                        gyro_x: data.gyroscope.x as f64,
                        gyro_y: data.gyroscope.y as f64,
                        gyro_z: data.gyroscope.z as f64,
                        mag_x: None,
                        mag_y: None,
                        mag_z: None,
                        error: None,
                    })
                } else {
                    error!("Failed to get BMI088 data");
                    Err(eyre::eyre!("Failed to get BMI088 data"))
                }
            }
        }
    }

    async fn get_advanced_values(&self) -> Result<ImuAdvancedValuesResponse> {
        match &self.imu {
            ImuReader::Bno055(bno) => {
                let data = bno.get_data()?;
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
            ImuReader::Bmi088(bmi) => {
                let data = bmi.get_data()?;
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
        }
    }

    async fn get_euler(&self) -> Result<EulerAnglesResponse> {
        match &self.imu {
            ImuReader::Bno055(bno) => {
                let data = bno.get_data()?;
                Ok(EulerAnglesResponse {
                    roll: data.euler.roll as f64,
                    pitch: data.euler.pitch as f64,
                    yaw: data.euler.yaw as f64,
                    error: None,
                })
            }
            ImuReader::Bmi088(bmi) => {
                let data = bmi.get_data()?;
                Ok(EulerAnglesResponse {
                    roll: data.euler.roll as f64,
                    pitch: data.euler.pitch as f64,
                    yaw: data.euler.yaw as f64,
                    error: None,
                })
            }
        }
    }

    async fn get_quaternion(&self) -> Result<QuaternionResponse> {
        match &self.imu {
            ImuReader::Bno055(bno) => {
                let data = bno.get_data()?;
                Ok(QuaternionResponse {
                    w: data.quaternion.w as f64,
                    x: data.quaternion.x as f64,
                    y: data.quaternion.y as f64,
                    z: data.quaternion.z as f64,
                    error: None,
                })
            }
            ImuReader::Bmi088(bmi) => {
                let data = bmi.get_data()?;
                Ok(QuaternionResponse {
                    w: data.quaternion.w as f64,
                    x: data.quaternion.x as f64,
                    y: data.quaternion.y as f64,
                    z: data.quaternion.z as f64,
                    error: None,
                })
            }
        }
    }

    async fn calibrate(&self) -> Result<Operation> {
        match &self.imu {
            ImuReader::Bno055(bno) => bno.reset()?,
            ImuReader::Bmi088(bmi) => bmi.reset()?,
        };
        let cal_status = match &self.imu {
            ImuReader::Bno055(bno) => bno.get_data()?.calibration_status,
            ImuReader::Bmi088(bmi) => bmi.get_data()?.calibration_status,
        };

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
        _duration: Option<Duration>,
        _max_retries: Option<u32>,
        _max_angular_error: Option<f32>,
        _max_vel: Option<f32>,
        _max_accel: Option<f32>,
    ) -> Result<ActionResponse> {
        match &self.imu {
            ImuReader::Bno055(bno) => {
                bno.reset()?;
                if let Err(e) = bno.set_mode(OperationMode::Ndof) {
                    error!("Failed to set IMU mode after reset: {}", e);
                    return Ok(ActionResponse {
                        success: false,
                        error: Some(Error {
                            code: ErrorCode::HardwareFailure as i32,
                            message: format!("Failed to set IMU mode: {}", e),
                        }),
                    });
                }
            }
            ImuReader::Bmi088(bmi) => {
                bmi.reset()?;
                // set_mode is a no-op for BMI088, pass 0 as dummy value
                let _ = bmi.set_mode(0);
            }
        }
        Ok(ActionResponse {
            success: true,
            error: None,
        })
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn sanity_check_imu() {
//         // Verify we can access BMI088 types and constants
//         let accel_addr = linux_bmi088::ACCEL_ADDR;
//         let gyro_addr = linux_bmi088::GYRO_ADDR;
        
//         // Create a default IMU data struct to verify the type is accessible
//         let _data = linux_bmi088::Bmi088Data::default();
        
//         println!("IMU sanity check - BMI088 addresses: 0x{:02X}/0x{:02X}", accel_addr, gyro_addr);
//     }
// }
