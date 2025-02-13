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
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tracing::{debug, error, info, warn};

pub struct ZBotBMI088 {
    imu: Bmi088Reader,
    /// Do fusion / correction logic in f32
    axis_correction: Rotation3<f32>,
    /// Complementary filter for orientation estimation.
    comp_filter: Arc<Mutex<ComplementaryFilter>>,
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
                    "BMI088 accelerometer reading is non-zero. Good start. Took {:?}",
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
                // Initialize the complementary filter
                let comp_filter = ComplementaryFilter::new(0.90);
                return Ok(Self {
                    imu,
                    axis_correction,
                    comp_filter: Arc::new(Mutex::new(comp_filter)),
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

    /// Helper function that reads sensor data, applies the fixed axis correction,
    /// and updates the complementary filter. Returns the current orientation quaternion.
    fn update_orientation(&self) -> Result<UnitQuaternion<f32>> {
        let data = self.imu.get_data()?;

        // Convert raw accelerometer data from g to m/s².
        let raw_accel = Vector3::new(
            data.accelerometer.x * 9.81_f32,
            data.accelerometer.y * 9.81_f32,
            data.accelerometer.z * 9.81_f32,
        );
        // Gyroscope data are in deg/s.
        let raw_gyro = Vector3::new(data.gyroscope.x, data.gyroscope.y, data.gyroscope.z);
        // Apply the fixed axis correction.
        let corrected_accel = self.axis_correction * raw_accel;
        let corrected_gyro = self.axis_correction * raw_gyro;

        // Update the complementary filter with corrected gyro (in deg/s) and accel.
        let mut filter = self.comp_filter.lock().unwrap();
        let q = filter.update(corrected_gyro, corrected_accel);
        Ok(q)
    }
}

/// A simple complementary filter for estimating orientation.
/// The filter fuses the gyroscope (high-pass) and accelerometer (low-pass) data to compute roll and pitch.
/// Yaw is integrated from the gyro alone.
pub struct ComplementaryFilter {
    roll: f32,
    pitch: f32,
    yaw: f32,
    last_update: Option<Instant>,
    /// The blending coefficient; typically close to 1 (e.g., 0.98) to favor gyro integration.
    alpha: f32,
}

impl ComplementaryFilter {
    pub fn new(alpha: f32) -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            last_update: None,
            alpha,
        }
    }

    /// Update the filter with new measurements.
    /// `gyro` is in deg/s and `accel` is in m/s².
    pub fn update(&mut self, gyro: Vector3<f32>, accel: Vector3<f32>) -> UnitQuaternion<f32> {
        let now = Instant::now();
        // Use dt = 0.01 if this is the first update.
        let dt = self
            .last_update
            .map_or(0.01, |last| now.duration_since(last).as_secs_f32());
        self.last_update = Some(now);

        // Convert gyro readings from deg/s to rad/s.
        let gyro_rad = gyro * (std::f32::consts::PI / 180.0);

        // Integrate gyro readings.
        let roll_gyro = self.roll + gyro_rad.x * dt;
        let pitch_gyro = self.pitch + gyro_rad.y * dt;
        let yaw_gyro = self.yaw + gyro_rad.z * dt;

        // Calculate roll and pitch from accelerometer.
        let roll_acc = accel.y.atan2(accel.z);
        let pitch_acc = (-accel.x).atan2((accel.y.powi(2) + accel.z.powi(2)).sqrt());

        // Complementary filter blending.
        self.roll = self.alpha * roll_gyro + (1.0 - self.alpha) * roll_acc;
        self.pitch = self.alpha * pitch_gyro + (1.0 - self.alpha) * pitch_acc;
        self.yaw = yaw_gyro; // Yaw: no accelerometer correction

        UnitQuaternion::from_euler_angles(self.roll, self.pitch, self.yaw)
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

        // Apply the fixed correction.
        let corrected_accel = self.axis_correction * raw_accel;
        let corrected_gyro = self.axis_correction * raw_gyro;
        let corrected_mag = self.axis_correction * raw_mag;

        // Return the corrected sensor values in f64.
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
        // Update the filter using the latest sensor data and retrieve the orientation quaternion.
        let q = self.update_orientation()?;
        let (roll, pitch, yaw) = q.euler_angles();
        Ok(EulerAnglesResponse {
            roll: roll as f64,
            pitch: pitch as f64,
            yaw: yaw as f64,
            error: None,
        })
    }

    async fn get_quaternion(&self) -> Result<QuaternionResponse> {
        let q = self.update_orientation()?;
        Ok(QuaternionResponse {
            w: q.w as f64,
            x: q.i as f64,
            y: q.j as f64,
            z: q.k as f64,
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
