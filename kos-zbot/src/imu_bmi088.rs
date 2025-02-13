use async_trait::async_trait;
use eyre::Result;
use imu::bmi088::Bmi088Reader;
use kos::{
    hal::{
        EulerAnglesResponse, ImuAdvancedValuesResponse, ImuValuesResponse, Operation,
        QuaternionResponse, IMU,
    },
    kos_proto::common::{ActionResponse, Error, ErrorCode},
};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use std::time::{Duration, Instant};
use tracing::{debug, error, info, warn};

pub struct ZBotBMI088 {
    imu: Bmi088Reader,
    /// The fixed correction from sensor space to robot space,
    /// stored in single-precision.
    axis_correction: Rotation3<f32>,
}

impl ZBotBMI088 {
    /// Attempts to initialize the BMI088 sensor on the specified I2C bus.
    ///
    /// Often the Bmi088 has a "bad start", in which case it needs to be reinitialized.
    /// If the accelerometer reading is zero after 0.1 seconds, the sensor is reinitialized.
    /// This process is repeated until the sensor is successfully initialized or 5 seconds have elapsed.
    pub fn new(i2c_bus: &str) -> Result<Self> {
        info!("Initializing BMI088 on bus: {}", i2c_bus);
        let overall_start = Instant::now();
        loop {
            let imu = Bmi088Reader::new(i2c_bus)?;
            std::thread::sleep(Duration::from_millis(100));
            let sample = imu.get_data()?;
            if sample.accelerometer.x != 0.0
                || sample.accelerometer.y != 0.0
                || sample.accelerometer.z != 0.0
            {
                info!(
                    "BMI088 accelerometer reading is non-zero; good start. Took {:?}",
                    overall_start.elapsed()
                );
                // The IMU is mounted such that:
                //   Sensor X: down, Sensor Y: backward, Sensor Z: left.
                // Our robot expects:
                //   Robot X: forward, Robot Y: left, Robot Z: up.
                // Therefore we want:
                //   Robot X = - (Sensor Y)
                //   Robot Y =   (Sensor Z)
                //   Robot Z = - (Sensor X)
                //
                // The corresponding rotation matrix is defined below.
                let axis_correction = Rotation3::from_matrix(&Matrix3::new(
                    0.0_f32, -1.0_f32, 0.0_f32, 0.0_f32, 0.0_f32, 1.0_f32, -1.0_f32, 0.0_f32,
                    0.0_f32,
                ));
                debug!("Using fixed axis correction: {:?}", axis_correction);
                return Ok(Self {
                    imu,
                    axis_correction,
                });
            } else {
                warn!("BMI088 accelerometer reading is zero after 0.1 seconds; reinitializing");
            }
            if overall_start.elapsed() >= Duration::from_secs(5) {
                warn!("BMI088 failed to initialize properly after 5 seconds; giving up");
                return Err(eyre::eyre!(
                    "BMI088 failed to initialize within 5 seconds; no non-zero acceleration reading"
                ));
            }
        }
    }
}

#[async_trait]
impl IMU for ZBotBMI088 {
    async fn get_values(&self) -> Result<ImuValuesResponse> {
        let data = self.imu.get_data()?;

        // Build raw sensor vectors in f32.
        let raw_accel = Vector3::new(
            data.accelerometer.x * 9.81_f32,
            data.accelerometer.y * 9.81_f32,
            data.accelerometer.z * 9.81_f32,
        );
        let raw_gyro = Vector3::new(data.gyroscope.x, data.gyroscope.y, data.gyroscope.z);
        let raw_mag = Vector3::new(
            data.magnetometer.x,
            data.magnetometer.y,
            data.magnetometer.z,
        );

        // Apply the fixed correction (all in f32).
        let corrected_accel = self.axis_correction * raw_accel;
        let corrected_gyro = self.axis_correction * raw_gyro;
        let corrected_mag = self.axis_correction * raw_mag;

        // Convert to f64 when returning.
        Ok(ImuValuesResponse {
            accel_x: corrected_accel.x as f64,
            accel_y: corrected_accel.y as f64,
            accel_z: corrected_accel.z as f64,
            gyro_x: corrected_gyro.x as f64,
            gyro_y: corrected_gyro.y as f64,
            gyro_z: corrected_gyro.z as f64,
            mag_x: Some(corrected_mag.x as f64),
            mag_y: Some(corrected_mag.y as f64),
            mag_z: Some(corrected_mag.z as f64),
            error: None,
        })
    }

    async fn get_advanced_values(&self) -> Result<ImuAdvancedValuesResponse> {
        let data = self.imu.get_data()?;
        let raw_lin_acc = Vector3::new(
            data.linear_acceleration.x * 9.81_f32,
            data.linear_acceleration.y * 9.81_f32,
            data.linear_acceleration.z * 9.81_f32,
        );
        let raw_grav = Vector3::new(
            data.gravity.x * 9.81_f32,
            data.gravity.y * 9.81_f32,
            data.gravity.z * 9.81_f32,
        );

        let corrected_lin_acc = self.axis_correction * raw_lin_acc;
        let corrected_grav = self.axis_correction * raw_grav;

        Ok(ImuAdvancedValuesResponse {
            lin_acc_x: Some(corrected_lin_acc.x as f64),
            lin_acc_y: Some(corrected_lin_acc.y as f64),
            lin_acc_z: Some(corrected_lin_acc.z as f64),
            grav_x: Some(corrected_grav.x as f64),
            grav_y: Some(corrected_grav.y as f64),
            grav_z: Some(corrected_grav.z as f64),
            temp: Some(data.temperature as f64),
            error: None,
        })
    }

    async fn get_euler(&self) -> Result<EulerAnglesResponse> {
        let data = self.imu.get_data()?;
        // Build the sensor quaternion in f32.
        let sensor_quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            data.quaternion.w,
            data.quaternion.x,
            data.quaternion.y,
            data.quaternion.z,
        ));
        // Convert our axis correction into a quaternion.
        let correction_quat = UnitQuaternion::from_rotation_matrix(&self.axis_correction);
        // Apply the correction.
        let corrected_quat = correction_quat * sensor_quat;
        let (roll, pitch, yaw) = corrected_quat.euler_angles();
        Ok(EulerAnglesResponse {
            roll: roll as f64,
            pitch: pitch as f64,
            yaw: yaw as f64,
            error: None,
        })
    }

    async fn get_quaternion(&self) -> Result<QuaternionResponse> {
        let data = self.imu.get_data()?;
        let sensor_quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            data.quaternion.w,
            data.quaternion.x,
            data.quaternion.y,
            data.quaternion.z,
        ));
        let correction_quat = UnitQuaternion::from_rotation_matrix(&self.axis_correction);
        let corrected_quat = correction_quat * sensor_quat;
        Ok(QuaternionResponse {
            w: corrected_quat.w as f64,
            x: corrected_quat.i as f64,
            y: corrected_quat.j as f64,
            z: corrected_quat.k as f64,
            error: None,
        })
    }

    async fn calibrate(&self) -> Result<Operation> {
        info!("Starting BMI088 calibration");
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
                // BMI088 uses u8 for mode, 0x0C is equivalent to NDOF mode
                if let Err(e) = self.imu.set_mode(0x0C) {
                    error!("Failed to set BMI088 mode after reset: {}", e);
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
                error!("Failed to zero BMI088: {}", e);
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
