use eyre::Result;
use std::os::raw::{c_int, c_short, c_uchar, c_ushort, c_uint};
use crate::hal::{ServoInfo, ServoData, ServoMultipleWriteCommand, ServoMode, ServoDirection, ServoRegister, MemoryLockState, TorqueMode, MAX_SERVOS};
use std::sync::{Arc, Mutex};
use std::fmt;

#[link(name = "sts3215")]
extern "C" {
    fn servo_init() -> c_int;
    fn servo_deinit();
    fn servo_write(id: c_uchar, address: c_uchar, data: *const c_uchar, length: c_uchar) -> c_int;
    fn servo_read(id: c_uchar, address: c_uchar, length: c_uchar, data: *mut c_uchar) -> c_int;
    fn servo_move(id: c_uchar, position: c_short, time: c_ushort, speed: c_ushort) -> c_int;
    fn enable_servo_readout() -> c_int;
    fn disable_servo_readout() -> c_int;
    fn enable_servo_movement() -> c_int;
    fn disable_servo_movement() -> c_int;
    fn set_servo_mode(id: c_uchar, mode: c_uchar) -> c_int;
    fn set_servo_speed(id: c_uchar, speed: c_ushort, direction: c_int) -> c_int;
    fn servo_read_info(id: c_uchar, info: *mut ServoInfo) -> c_int;
    fn read_servo_positions(servo_data: *mut ServoData) -> c_int;
    fn servo_write_multiple(cmd: *const ServoMultipleWriteCommand) -> c_int;
}

#[derive(Debug)]
pub struct Servo {
    _private: (), // Prevent direct construction
}

impl Servo {
    pub fn new() -> Result<Self> {
        let result = unsafe { servo_init() };
        if result != 0 {
            eyre::bail!("Failed to initialize servo");
        }
        Ok(Servo { _private: () })
    }

    pub fn write(&self, id: u8, register: ServoRegister, data: &[u8]) -> Result<()> {
        let _result = unsafe { servo_write(id, register.clone() as u8, data.as_ptr(), data.len() as c_uchar) };
        let result = unsafe { servo_write(id, register as u8, data.as_ptr(), data.len() as c_uchar) };

        if result != 0 {
            eyre::bail!("Failed to write to servo");
        }
        Ok(())
    }

    pub fn read(&self, id: u8, register: ServoRegister, length: u8) -> Result<Vec<u8>> {
        let mut data = vec![0u8; length as usize];
        let result = unsafe { servo_read(id, register as u8, length, data.as_mut_ptr()) };
        if result != 0 {
            eyre::bail!("Failed to read from servo");
        }
        Ok(data)
    }

    pub fn move_servo(&self, id: u8, position: i16, time: u16, speed: u16) -> Result<()> {
        let result = unsafe { servo_move(id, position, time, speed) };
        if result != 0 {
            eyre::bail!("Failed to move servo");
        }
        Ok(())
    }

    pub fn enable_readout(&self) -> Result<()> {
        let result = unsafe { enable_servo_readout() };
        if result != 0 {
            eyre::bail!("Failed to enable servo readout");
        }
        Ok(())
    }

    pub fn disable_readout(&self) -> Result<()> {
        let result = unsafe { disable_servo_readout() };
        if result != 0 {
            eyre::bail!("Failed to disable servo readout");
        }
        Ok(())
    }

    pub fn enable_movement(&self) -> Result<()> {
        let result = unsafe { enable_servo_movement() };
        if result != 0 {
            eyre::bail!("Failed to enable servo movement");
        }
        Ok(())
    }

    pub fn disable_movement(&self) -> Result<()> {
        let result = unsafe { disable_servo_movement() };
        if result != 0 {
            eyre::bail!("Failed to disable servo movement");
        }
        Ok(())
    }

    pub fn set_mode(&self, id: u8, mode: ServoMode) -> Result<()> {
        let result = unsafe { set_servo_mode(id, mode as u8) };
        if result != 0 {
            eyre::bail!("Failed to set servo mode");
        }
        Ok(())
    }

    pub fn set_speed(&self, id: u8, speed: u16, direction: ServoDirection) -> Result<()> {
        let direction = if direction == ServoDirection::Clockwise { 1 } else { -1 };
        let result = unsafe { set_servo_speed(id, speed, direction as i32) };
        if result != 0 {
            eyre::bail!("Failed to set servo speed");
        }
        Ok(())
    }

    pub fn read_info(&self, id: u8) -> Result<ServoInfo> {
        let mut info = ServoInfo {
            torque_switch: 0,
            acceleration: 0,
            target_location: 0,
            running_time: 0,
            running_speed: 0,
            torque_limit: 0,
            reserved1: [0; 6],
            lock_mark: 0,
            current_location: 0,
            current_speed: 0,
            current_load: 0,
            current_voltage: 0,
            current_temperature: 0,
            async_write_flag: 0,
            servo_status: 0,
            mobile_sign: 0,
            reserved2: [0; 2],
            current_current: 0,
        };
        let result = unsafe { servo_read_info(id, &mut info) };
        if result != 0 {
            eyre::bail!("Failed to read servo info");
        }
        Ok(info)
    }

    pub fn read_continuous(&self) -> Result<ServoData> {
        let mut data = ServoData {
            servo: [ServoInfo {
                torque_switch: 0,
                acceleration: 0,
                target_location: 0,
                running_time: 0,
                running_speed: 0,
                torque_limit: 0,
                reserved1: [0; 6],
                lock_mark: 0,
                current_location: 0,
                current_speed: 0,
                current_load: 0,
                current_voltage: 0,
                current_temperature: 0,
                async_write_flag: 0,
                servo_status: 0,
                mobile_sign: 0,
                reserved2: [0; 2],
                current_current: 0,
            }; MAX_SERVOS],
            task_run_count: 0,
        };
        let result = unsafe { read_servo_positions(&mut data) };
        if result != 0 {
            eyre::bail!("Failed to read continuous servo data");
        }
        Ok(data)
    }

    pub fn write_multiple(&self, cmd: &ServoMultipleWriteCommand) -> Result<()> {
        let result = unsafe { servo_write_multiple(cmd) };
        if result != 0 {
            eyre::bail!("Failed to write multiple servo positions");
        }
        Ok(())
    }

    pub fn read_pid(&self, id: u8) -> Result<(u8, u8, u8)> {
        let p = self.read(id, ServoRegister::PProportionalCoeff, 1)?[0];
        let i = self.read(id, ServoRegister::IIntegralCoeff, 1)?[0];
        let d = self.read(id, ServoRegister::DDifferentialCoeff, 1)?[0];
        Ok((p, i, d))
    }

    pub fn set_pid(&self, id: u8, p: u8, i: u8, d: u8) -> Result<()> {
        // Unlock flash
        self.write(id, ServoRegister::LockMark, &[MemoryLockState::Unlocked as u8])?;

        // Set PID parameters
        self.write(id, ServoRegister::PProportionalCoeff, &[p])?;
        self.write(id, ServoRegister::IIntegralCoeff, &[i])?;
        self.write(id, ServoRegister::DDifferentialCoeff, &[d])?;

        // Lock flash
        self.write(id, ServoRegister::LockMark, &[MemoryLockState::Locked as u8])?;

        Ok(())
    }

    pub fn set_memory_lock(&self, id: u8, state: MemoryLockState) -> Result<()> {
        self.write(id, ServoRegister::LockMark, &[state as u8])
    }

    pub fn read_angle_limits(&self, id: u8) -> Result<(i16, i16)> {
        let min_limit = i16::from_le_bytes(self.read(id, ServoRegister::MinAngleLimit, 2)?.try_into().unwrap());
        let max_limit = i16::from_le_bytes(self.read(id, ServoRegister::MaxAngleLimit, 2)?.try_into().unwrap());
        Ok((min_limit, max_limit))
    }

    pub fn set_torque_mode(&self, id: u8, mode: TorqueMode) -> Result<()> {
        self.write(id, ServoRegister::TorqueSwitch, &[mode as u8])
    }

    pub fn write_servo_memory(&self, id: u8, register: ServoRegister, value: u16) -> Result<()> {
        let data = [(value & 0xFF) as u8, ((value >> 8) & 0xFF) as u8];
        self.write(id, register, &data)
    }

    pub fn scan(&self, id: u8) -> Result<bool> {
        // Try to read the servo ID from memory address 0x5 (ServoRegister::ID)
        match self.read(id, ServoRegister::ID, 1) {
            Ok(data) if data.len() == 1 && data[0] == id => Ok(true),
            Ok(_) => Ok(false), // Received data, but it doesn't match the ID
            Err(_) => Ok(false), // No response, assume no servo at this ID
        }
    }

    pub fn degrees_to_raw(degrees: f32) -> u16 {
        // Ensure the input is within the valid range
        let clamped_degrees = degrees.max(-180.0).min(180.0);
        
        // Convert degrees to raw value
        let raw = (clamped_degrees + 180.0) / 360.0 * 4096.0;
        
        // Round to nearest integer and ensure it's within the valid range
        raw.round().max(0.0).min(4095.0) as u16
    }

    pub fn raw_to_degrees(raw: u16) -> f32 {
        // Ensure the input is within the valid range
        let clamped_raw = raw.max(0).min(4095);
        
        // Convert raw value to degrees
        let degrees = (clamped_raw as f32 / 4096.0) * 360.0 - 180.0;
        
        // Round to two decimal places
        (degrees * 100.0).round() / 100.0;

        // clamp to -180.0 to 180.0
        degrees.max(-180.0).min(180.0)
    }
}

impl Drop for Servo {
    fn drop(&mut self) {
        unsafe { servo_deinit() };
    }
}