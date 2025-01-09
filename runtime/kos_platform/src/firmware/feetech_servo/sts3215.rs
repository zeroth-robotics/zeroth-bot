use crate::firmware::feetech::{FeetechActuatorInfo, ServoInfo, FeetechActuator, };

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
}

impl FeetechActuator for Sts3215 {
    fn id(&self) -> u8 {
        self.id
    }

    fn info(&self) -> FeetechActuatorInfo {
        self.info
    }

    fn set_position(&mut self, position_deg: f32) {
        unimplemented!()
    }
    fn set_speed(&mut self, speed_deg_per_s: f32) {
        unimplemented!()
    }
    fn enable_torque(&mut self) {
        self.info.torque_enabled = true;
        unimplemented!()
    }
    fn disable_torque(&mut self) {
        unimplemented!()
    }
    fn change_id(&mut self, id: u8) {
        unimplemented!()
    }
    fn update_info(&mut self, info: &ServoInfo) {
        self.info.id = self.id;
        self.info.position_deg = self.raw_to_degrees(info.current_location as u16);
        self.info.speed_deg_per_s = {
            let speed_raw = info.current_speed as u16;
            let speed_magnitude = speed_raw & 0x7FFF;
            let speed_sign = if speed_raw & 0x8000 != 0 { -1.0 } else { 1.0 };
            speed_sign * (self.raw_to_degrees(speed_magnitude))
        };
        self.info.load_percent = info.current_load as f32 / 100.0;
        self.info.voltage_v = info.current_voltage as f32 / 10.0;
        self.info.current_ma = info.current_current as f32 / 100.0 * 6.5;
        self.info.temperature_c = info.current_temperature as f32;
    }

    fn degrees_to_raw(&self, degrees: f32) -> u16 {
        (degrees / 360.0 * 4096.0) as u16
    }

    fn raw_to_degrees(&self, raw: u16) -> f32 {
        raw as f32 / 4096.0 * 360.0
    }
}
