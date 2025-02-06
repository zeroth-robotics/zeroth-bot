use crate::firmware::feetech::{
    feetech_read, feetech_write, FeetechActuator, FeetechActuatorInfo, FeetechOperationMode,
    ServoInfo,
};
use eyre::{eyre, Result};
use std::thread;
use std::time::Duration;
use tracing::{debug, trace, warn};

#[allow(dead_code)]
enum Sts3215Register {
    ID = 0x05,
    MinAngle = 0x09,
    MaxAngle = 0x0B,
    Offset = 0x1F,
    PProportionalCoeff = 0x15,
    DDifferentialCoeff = 0x16,
    IIntegralCoeff = 0x17,
    OperationMode = 0x21,
    TorqueSwitch = 0x28,
    TargetLocation = 0x2A,
    RunningTime = 0x2C,
    RunningSpeed = 0x2E,
    Acceleration = 0x29,
    TorqueLimit = 0x30,
    LockMark = 0x37,
    CurrentLocation = 0x38,
    CurrentSpeed = 0x3A,
    CurrentLoad = 0x3C,
    CurrentVoltage = 0x3E,
    CurrentTemperature = 0x3F,
    AsyncWriteFlag = 0x40,
    ServoStatus = 0x41,
    MobileSign = 0x42,
    CurrentCurrent = 0x45,
}

#[derive(Debug, Clone)]
pub struct Sts3215 {
    pub id: u8,
    pub info: FeetechActuatorInfo,
}

impl Sts3215 {
    pub fn new(id: u8) -> Self {
        Self {
            id,
            info: FeetechActuatorInfo::default(),
        }
    }

    fn unlock_eeprom(&mut self) -> Result<()> {
        feetech_write(self.id, Sts3215Register::LockMark as u8, &[0x00])
            .map_err(|e| eyre!("Failed to unlock EEPROM: {}", e))?;
        Ok(())
    }

    pub fn lock_eeprom(&mut self) -> Result<()> {
        feetech_write(self.id, Sts3215Register::LockMark as u8, &[0x01])
            .map_err(|e| eyre!("Failed to lock EEPROM: {}", e))?;
        Ok(())
    }

    pub fn check_id(&mut self) -> Result<()> {
        let id = feetech_read(self.id, Sts3215Register::ID as u8, 1)?[0];
        if id != self.id {
            return Err(eyre!("Servo ID mismatch: expected {}, got {}", self.id, id));
        }
        Ok(())
    }

    fn status_to_faults(&self, status: u8) -> Vec<String> {
        let mut faults = Vec::new();
        if status & (1 << 0) != 0 {
            faults.push("Voltage".to_string());
        }
        if status & (1 << 1) != 0 {
            faults.push("Sensor".to_string());
        }
        if status & (1 << 2) != 0 {
            faults.push("Temperature".to_string());
        }
        if status & (1 << 3) != 0 {
            faults.push("Current".to_string());
        }
        if status & (1 << 4) != 0 {
            faults.push("Angle".to_string());
        }
        if status & (1 << 5) != 0 {
            faults.push("Overload".to_string());
        }
        faults
    }
}

impl FeetechActuator for Sts3215 {
    fn id(&self) -> u8 {
        self.id
    }

    fn info(&self) -> FeetechActuatorInfo {
        self.info.clone()
    }

    fn set_position(&mut self, position_deg: f32) -> Result<()> {
        let raw = self.degrees_to_raw(position_deg, 180.0);
        feetech_write(self.id, Sts3215Register::TargetLocation as u8, &[raw as u8])
            .map_err(|e| eyre!("Failed to set position: {}", e))?;
        Ok(())
    }

    fn set_speed(&mut self, speed_deg_per_s: f32) -> Result<()> {
        let abs_speed = speed_deg_per_s.abs();
        let sign: u16 = if speed_deg_per_s < 0.0 {
            0x8000
        } else {
            0x0000
        };
        let speed_raw = self.degrees_to_raw(abs_speed, 0.0) | sign;
        debug!("Setting speed: {} -> {}", speed_deg_per_s, speed_raw);
        feetech_write(
            self.id,
            Sts3215Register::RunningSpeed as u8,
            &[(speed_raw & 0xFF) as u8, ((speed_raw >> 8) & 0xFF) as u8],
        )
        .map_err(|e| eyre!("Failed to set speed: {}", e))?;
        Ok(())
    }
    

    fn set_acceleration(&mut self, accel_deg_per_s2: f32) -> Result<()> { 
        let accel_scaled = self.degrees_to_raw(accel_deg_per_s2, 0.0) as f32 / 100.0;
        let accel_raw = accel_scaled.clamp(0.0, 254.0) as u8;    
        debug!("Setting acceleration: {} deg/s² -> {} register units", accel_deg_per_s2, accel_raw);
        
        feetech_write(
            self.id,
            Sts3215Register::Acceleration as u8,
            &[accel_raw],
        )
        .map_err(|e| eyre!("Failed to set acceleration: {}", e))?;
        
        Ok(())
    }
    
    
    fn set_operation_mode(&mut self, mode: FeetechOperationMode) -> Result<()> {
        match mode {
            FeetechOperationMode::PositionControl => {
                feetech_write(self.id, Sts3215Register::OperationMode as u8, &[0x00])
                    .map_err(|e| eyre!("Failed to set operation mode: {}", e))?
            }
            FeetechOperationMode::SpeedControl => {
                feetech_write(self.id, Sts3215Register::OperationMode as u8, &[0x01])
                    .map_err(|e| eyre!("Failed to set operation mode: {}", e))?
            }
            FeetechOperationMode::TorqueControl => {
                return Err(eyre!("Torque control is not supported for Sts3215"))
            }
        }
        Ok(())
    }

    fn enable_torque(&mut self) -> Result<()> {
        self.info.torque_enabled = true;
        feetech_write(self.id, Sts3215Register::TorqueSwitch as u8, &[0x01])
            .map_err(|e| eyre!("Failed to enable torque: {}", e))?;
        Ok(())
    }

    fn disable_torque(&mut self) -> Result<()> {
        self.info.torque_enabled = false;
        feetech_write(self.id, Sts3215Register::TorqueSwitch as u8, &[0x00])
            .map_err(|e| eyre!("Failed to disable torque: {}", e))?;
        Ok(())
    }

    fn change_id(&mut self, id: u8) -> Result<()> {
        // TODO: verification and prechecks
        self.unlock_eeprom()?;
        feetech_write(self.id, Sts3215Register::ID as u8, &[id])
            .map_err(|e| eyre!("Failed to change ID: {}", e))?;
        self.lock_eeprom()?;
        self.id = id;
        Ok(())
    }

    fn update_info(&mut self, info: &ServoInfo) {
        self.info.id = self.id;
        self.info.position_deg = self.raw_to_degrees(info.current_location as u16, 180.0);
        self.info.speed_deg_per_s = {
            let speed_raw = info.current_speed as u16;
            let speed_magnitude = speed_raw & 0x7FFF;
            let speed_sign = if speed_raw & 0x8000 != 0 { -1.0 } else { 1.0 };
            speed_sign * self.raw_to_degrees(speed_magnitude, 0.0)
        };
        self.info.load_percent = info.current_load as f32 / 100.0;
        self.info.voltage_v = info.current_voltage as f32 / 10.0;
        self.info.current_ma = info.current_current as f32 / 100.0 * 6.5;
        self.info.temperature_c = info.current_temperature as f32;

        let current_time_ms = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis() as u32;

        if self.info.last_read_ms == info.last_read_ms
            && current_time_ms - self.info.last_read_local_ms > 100
        {
            if self.info.online {
                warn!("Servo {} is not responding", self.id);
            }
            self.info.online = false;
        } else if self.info.last_read_ms != info.last_read_ms {
            self.info.online = true;
            self.info.last_read_local_ms = current_time_ms;
            self.info.last_read_ms = info.last_read_ms;
        }

        self.info.faults = self.status_to_faults(info.servo_status);
    }

    fn degrees_to_raw(&self, degrees: f32, offset: f32) -> u16 {
        ((degrees + offset) / 360.0 * 4096.0) as u16
    }

    fn raw_to_degrees(&self, raw: u16, offset: f32) -> f32 {
        raw as f32 / 4096.0 * 360.0 - offset
    }

    fn set_pid(&mut self, p: Option<f32>, i: Option<f32>, d: Option<f32>) -> Result<()> {
        self.unlock_eeprom()?;
        if let Some(p) = p {
            feetech_write(
                self.id,
                Sts3215Register::PProportionalCoeff as u8,
                &[p as u8],
            )?;
        }
        if let Some(i) = i {
            feetech_write(self.id, Sts3215Register::IIntegralCoeff as u8, &[i as u8])?;
        }
        if let Some(d) = d {
            feetech_write(
                self.id,
                Sts3215Register::DDifferentialCoeff as u8,
                &[d as u8],
            )?;
        }
        self.lock_eeprom()?;
        Ok(())
    }

    fn get_current(&self) -> Result<f32> {
        let current_raw = feetech_read(self.id, Sts3215Register::CurrentCurrent as u8, 2)?;
        let current = u16::from_le_bytes(current_raw.clone().try_into().unwrap()) as f32 * 6.5;
        println!(
            "Current: {}, raw: {:?}",
            current,
            u16::from_le_bytes(current_raw.try_into().unwrap())
        );
        Ok(current)
    }

    fn write_calibration_data(
        &mut self,
        min_angle: f32,
        max_angle: f32,
        offset: f32,
    ) -> Result<()> {
        let min_raw = self.degrees_to_raw(min_angle, 180.0) as i32;
        let mut max_raw = self.degrees_to_raw(max_angle, 180.0) as i32;

        if max_raw < min_raw {
            max_raw += 4096;
        }

        let servo_offset = min_raw + (max_raw - min_raw) / 2 - 2048;

        let servo_offset = if servo_offset < 0 {
            servo_offset.abs() as u16 | 0x800
        } else {
            servo_offset as u16
        };

        let servo_offset = servo_offset + self.degrees_to_raw(offset, 0.0);

        trace!(
            "Writing calibration, offset: {}, min_angle: {}, max_angle: {}",
            servo_offset,
            min_raw,
            max_raw
        );

        let min_raw = 2048 - (max_raw - min_raw) / 2;
        let max_raw = 2048 + (max_raw - min_raw) / 2;

        let min_raw: u16 = std::cmp::max(0, min_raw) as u16;
        let max_raw: u16 = std::cmp::min(4095, max_raw) as u16;

        trace!("unlocking EEPROM");
        self.unlock_eeprom()?;
        trace!("writing min angle");
        feetech_write(
            self.id,
            Sts3215Register::MinAngle as u8,
            &min_raw.to_le_bytes(),
        )?;
        trace!("writing max angle");
        feetech_write(
            self.id,
            Sts3215Register::MaxAngle as u8,
            &max_raw.to_le_bytes(),
        )?;
        trace!("writing offset");
        feetech_write(
            self.id,
            Sts3215Register::Offset as u8,
            &servo_offset.to_le_bytes(),
        )?;
        trace!("writing operation mode");
        feetech_write(self.id, Sts3215Register::OperationMode as u8, &[0x00])?;
        trace!("locking EEPROM");
        self.lock_eeprom()?;
        Ok(())
    }

    fn set_zero_position(&mut self) -> Result<()> {
        self.unlock_eeprom()?;
        feetech_write(self.id, Sts3215Register::MinAngle as u8, &[0x00, 0x00])?;
        thread::sleep(Duration::from_millis(10));
        feetech_write(self.id, Sts3215Register::MaxAngle as u8, &[0x0F, 0xFF])?;
        thread::sleep(Duration::from_millis(10));
        self.set_operation_mode(FeetechOperationMode::PositionControl)?;
        thread::sleep(Duration::from_millis(10));
        feetech_write(self.id, Sts3215Register::TorqueSwitch as u8, &[0x80])?;
        thread::sleep(Duration::from_millis(10));
        self.lock_eeprom()?;
        Ok(())
    }
}
