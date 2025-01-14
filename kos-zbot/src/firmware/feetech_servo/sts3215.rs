use crate::firmware::feetech::{feetech_write, FeetechActuator, FeetechActuatorInfo, ServoInfo};
use eyre::{eyre, Result};

#[allow(dead_code)]
enum Sts3215Register {
    ID = 0x05,
    PProportionalCoeff = 0x15,
    DDifferentialCoeff = 0x16,
    IIntegralCoeff = 0x17,
    TorqueSwitch = 0x28,
    TargetLocation = 0x2A,
    RunningTime = 0x2C,
    RunningSpeed = 0x2E,
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

#[derive(Debug, Clone, Copy)]
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
        feetech_write(self.id, Sts3215Register::LockMark as u8, &[0x00]).map_err(|e| eyre!("Failed to unlock EEPROM: {}", e))?;
        Ok(())
    }

    fn lock_eeprom(&mut self) -> Result<()> {
        feetech_write(self.id, Sts3215Register::LockMark as u8, &[0x01]).map_err(|e| eyre!("Failed to lock EEPROM: {}", e))?;
        Ok(())
    }
}

impl FeetechActuator for Sts3215 {
    fn id(&self) -> u8 {
        self.id
    }

    fn info(&self) -> FeetechActuatorInfo {
        self.info
    }

    fn set_position(&mut self, position_deg: f32) -> Result<()> {
        let raw = self.degrees_to_raw(position_deg);
        feetech_write(self.id, Sts3215Register::TargetLocation as u8, &[raw as u8]).map_err(|e| eyre!("Failed to set position: {}", e))?;
        Ok(())
    }

    fn set_speed(&mut self, _speed_deg_per_s: f32) -> Result<()> {
        todo!("Implement speed control for ST3215 e.g. for calibration")
    }

    fn enable_torque(&mut self) -> Result<()> {
        self.info.torque_enabled = true;
        feetech_write(self.id, Sts3215Register::TorqueSwitch as u8, &[0x01]).map_err(|e| eyre!("Failed to enable torque: {}", e))?;
        Ok(())
    }

    fn disable_torque(&mut self) -> Result<()> {
        self.info.torque_enabled = false;
        feetech_write(self.id, Sts3215Register::TorqueSwitch as u8, &[0x00]).map_err(|e| eyre!("Failed to disable torque: {}", e))?;
        Ok(())
    }

    fn change_id(&mut self, id: u8) -> Result<()> {
        // TODO: verification and prechecks
        self.unlock_eeprom()?;
        feetech_write(self.id, Sts3215Register::ID as u8, &[id]).map_err(|e| eyre!("Failed to change ID: {}", e))?;
        self.lock_eeprom()?;
        self.id = id;
        Ok(())
    }

    fn update_info(&mut self, info: &ServoInfo) {
        self.info.id = self.id;
        self.info.position_deg = self.raw_to_degrees(info.current_location as u16);
        self.info.speed_deg_per_s = {
            let speed_raw = info.current_speed as u16;
            let speed_magnitude = speed_raw & 0x7FFF;
            let speed_sign = if speed_raw & 0x8000 != 0 { -1.0 } else { 1.0 };
            speed_sign * (self.raw_to_degrees(speed_magnitude) + 180.0)
        };
        self.info.load_percent = info.current_load as f32 / 100.0;
        self.info.voltage_v = info.current_voltage as f32 / 10.0;
        self.info.current_ma = info.current_current as f32 / 100.0 * 6.5;
        self.info.temperature_c = info.current_temperature as f32;
    }

    fn degrees_to_raw(&self, degrees: f32) -> u16 {
        ((degrees + 180.0) / 360.0 * 4096.0) as u16
    }

    fn raw_to_degrees(&self, raw: u16) -> f32 {
        raw as f32 / 4096.0 * 360.0 - 180.0
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
}
