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
use std::time::Duration;
use tracing::{debug, error, info};

pub struct ZBotBNO055 {
    imu: Bno055Reader,
}

impl ZBotBNO055 {
    pub fn new(i2c_bus: &str) -> Result<Self> {
        info!("Initializing BNO055 on bus: {}", i2c_bus);
        let imu = Bno055Reader::new(i2c_bus)?;
        Ok(Self { imu })
    }
}

#[async_trait]
impl IMU for ZBotBNO055 {
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
        info!("Starting BNO055 calibration");
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
        _duration: Option<Duration>,
        _max_retries: Option<u32>,
        _max_angular_error: Option<f32>,
        _max_vel: Option<f32>,
        _max_accel: Option<f32>,
    ) -> Result<ActionResponse> {
        match self.imu.reset() {
            Ok(_) => {
                if let Err(e) = self.imu.set_mode(OperationMode::Ndof) {
                    error!("Failed to set BNO055 mode after reset: {}", e);
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
                error!("Failed to zero BNO055: {}", e);
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
