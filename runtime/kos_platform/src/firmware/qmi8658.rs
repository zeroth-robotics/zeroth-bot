use eyre::Result;
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;
use std::error::Error as StdError;
use std::thread;
use std::time::Duration;

const QMI8658_SLAVE_ADDR_L: u8 = 0x6a;
const QMI8658_SLAVE_ADDR_H: u8 = 0x6b;

const ONE_G: f32 = 9.807;

// Register addresses
#[allow(dead_code)]
#[repr(u8)]
enum Register {
    WhoAmI = 0,
    Revision = 1,
    Ctrl1 = 2,
    Ctrl2 = 3,
    Ctrl3 = 4,
    Ctrl4 = 5,
    Ctrl5 = 6,
    Ctrl6 = 7,
    Ctrl7 = 8,
    Ctrl8 = 9,
    Ctrl9 = 10,
    // ... add other registers as needed
    StatusInt = 45,
    Status0 = 46,
    Status1 = 47,
    TimestampL = 48,
    TimestampM = 49,
    TimestampH = 50,
    TemperatureL = 51,
    TemperatureH = 52,
    AxL = 53,
    AxH = 54,
    AyL = 55,
    AyH = 56,
    AzL = 57,
    AzH = 58,
    GxL = 59,
    GxH = 60,
    GyL = 61,
    GyH = 62,
    GzL = 63,
    GzH = 64,
}

#[derive(Debug)]
pub struct ImuData {
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
}

pub struct QMI8658 {
    i2c: LinuxI2CDevice,
    acc_lsb_div: u16,
    gyro_lsb_div: u16,
}

impl QMI8658 {
    pub fn new(i2c_bus: &str) -> Result<Self> {
        let mut i2c = LinuxI2CDevice::new(i2c_bus, QMI8658_SLAVE_ADDR_H as u16)
            .map_err(|e| eyre::eyre!("Failed to create I2C device: {}", e))?;
        let chip_id = Self::read_reg(&mut i2c, Register::WhoAmI as u8)
            .map_err(|e| eyre::eyre!("Failed to read WHO_AM_I register: {}", e))?;

        Ok(Self {
            i2c,
            acc_lsb_div: 1 << 12, // Default 8g range
            gyro_lsb_div: 64,     // Default 512dps range
        })
    }

    pub fn init(&mut self) -> Result<()> {
        // Initialize the sensor with default settings
        self.write_reg(Register::Ctrl1 as u8, 0x60)
            .map_err(|e| eyre::eyre!("Failed to write Ctrl1: {}", e))?;

        // Configure accelerometer: 8g range, 1000Hz ODR
        self.write_reg(Register::Ctrl2 as u8, 0x23)
            .map_err(|e| eyre::eyre!("Failed to write Ctrl2: {}", e))?;

        // Configure gyroscope: 512dps range, 1000Hz ODR
        self.write_reg(Register::Ctrl3 as u8, 0x43)
            .map_err(|e| eyre::eyre!("Failed to write Ctrl3: {}", e))?;

        // Enable accelerometer and gyroscope
        self.write_reg(Register::Ctrl7 as u8, 0x03)
            .map_err(|e| eyre::eyre!("Failed to write Ctrl7: {}", e))?;

        Ok(())
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<()> {
        self.i2c
            .smbus_write_byte_data(reg, value)
            .map_err(|e| eyre::eyre!("I2C write failed: {}", e))?;
        Ok(())
    }

    fn read_reg(i2c: &mut LinuxI2CDevice, reg: u8) -> Result<u8> {
        i2c.smbus_read_byte_data(reg)
            .map_err(|e| eyre::eyre!("I2C read failed: {}", e))
    }

    fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<()> {
        for i in 0..buf.len() {
            buf[i] = self
                .i2c
                .smbus_read_byte_data(reg + i as u8)
                .map_err(|e| eyre::eyre!("I2C read failed at byte {}: {}", i, e))?;
        }
        Ok(())
    }

    pub fn read_data(&mut self) -> Result<ImuData> {
        let mut buf = [0u8; 12];
        self.read_bytes(Register::AxL as u8, &mut buf)?;

        // Convert accelerometer data
        let raw_acc_x = i16::from_le_bytes([buf[0], buf[1]]);
        let raw_acc_y = i16::from_le_bytes([buf[2], buf[3]]);
        let raw_acc_z = i16::from_le_bytes([buf[4], buf[5]]);

        // Convert gyroscope data
        let raw_gyro_x = i16::from_le_bytes([buf[6], buf[7]]);
        let raw_gyro_y = i16::from_le_bytes([buf[8], buf[9]]);
        let raw_gyro_z = i16::from_le_bytes([buf[10], buf[11]]);

        let imu_data = ImuData {
            acc_x: (raw_acc_x as f32 * ONE_G) / self.acc_lsb_div as f32,
            acc_y: (raw_acc_y as f32 * ONE_G) / self.acc_lsb_div as f32,
            acc_z: (raw_acc_z as f32 * ONE_G) / self.acc_lsb_div as f32,
            gyro_x: raw_gyro_x as f32 / self.gyro_lsb_div as f32,
            gyro_y: raw_gyro_y as f32 / self.gyro_lsb_div as f32,
            gyro_z: raw_gyro_z as f32 / self.gyro_lsb_div as f32,
        };

        Ok(imu_data)
    }
}
