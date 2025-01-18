use eyre::Result;
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;
use kos::{
    hal::LEDMatrix,
    kos_proto::{
        common::{ActionResponse, Error, ErrorCode},
        led_matrix::*,
    },
};
use std::sync::Mutex;

const DISPLAY_ADDR: u16 = 0x55;
const HEADER: [u8; 2] = [0xA5, 0x5A];

struct DisplayDriver<T: I2CDevice> {
    i2c: T,
}

impl<T: I2CDevice> DisplayDriver<T> {
    pub fn new(i2c: T) -> Self {
        Self { i2c }
    }

    pub fn write_region(
        &mut self,
        x: u8,
        y: u8,
        width: u8,
        height: u8,
        data: &[u8],
    ) -> std::result::Result<(), Box<dyn std::error::Error>>
    where
        T::Error: 'static,
    {
        // Validate input parameters
        if x >= 32 || y >= 16 || width == 0 || height == 0 || x + width > 32 || y + height > 16 {
            return Err("Invalid coordinates or dimensions".into());
        }

        // Calculate expected data length: (w/2 + w%2) * h
        let expected_len = ((width as usize / 2 + width as usize % 2) * height as usize) as usize;
        if data.len() != expected_len {
            return Err("Invalid data length".into());
        }

        // Construct message
        let mut message = Vec::with_capacity(6 + data.len());
        message.extend_from_slice(&HEADER);
        message.push(x);
        message.push(y);
        message.push(width);
        message.push(height);
        message.extend_from_slice(data);

        // Send data over I2C
        self.i2c.write(&message)?;
        Ok(())
    }
}

pub struct ZBotLEDMatrix {
    display: Mutex<DisplayDriver<LinuxI2CDevice>>,
}

impl ZBotLEDMatrix {
    pub fn new(i2c_path: &str) -> Result<Self> {
        let i2c = LinuxI2CDevice::new(i2c_path, DISPLAY_ADDR)
            .map_err(|e| eyre::eyre!("Failed to open I2C device: {}", e))?;
        Ok(Self {
            display: Mutex::new(DisplayDriver::new(i2c)),
        })
    }
}

#[tonic::async_trait]
impl LEDMatrix for ZBotLEDMatrix {
    async fn get_matrix_info(&self) -> Result<GetMatrixInfoResponse> {
        Ok(GetMatrixInfoResponse {
            width: 32,
            height: 16,
            color_capable: false,
            bits_per_pixel: 1,
            brightness_levels: 255,
            error: None,
        })
    }

    async fn write_buffer(&self, buffer: Vec<u8>) -> Result<ActionResponse> {
        if buffer.len() != 64 {
            // 32x16 pixels = 512 bits = 64 bytes
            return Ok(ActionResponse {
                success: false,
                error: Some(Error {
                    code: ErrorCode::InvalidArgument as i32,
                    message: "Buffer must be exactly 64 bytes (512 bits)".to_string(),
                }),
            });
        }

        // Convert 1bpp buffer to brightness values
        let mut brightness_data = Vec::with_capacity(32 * 16);
        for y in 0..16 {
            for x_byte in 0..4 {
                let byte_idx = y * 4 + x_byte;
                let byte = buffer[byte_idx];
                for bit_pair in (0..8).step_by(2) {
                    let bits = (byte >> (6 - bit_pair)) & 0b11;
                    let brightness = match bits {
                        0b10 => 0xF0, // First bit set
                        0b01 => 0x0F, // Second bit set
                        0b11 => 0xFF, // Both bits set
                        0b00 => 0x00, // No bits set
                        _ => unreachable!(),
                    };
                    brightness_data.push(brightness);
                }
            }
        }

        // Write to display using I2C
        let mut display = self
            .display
            .lock()
            .map_err(|_| eyre::eyre!("Failed to lock display"))?;
        display
            .write_region(0, 0, 32, 16, &brightness_data)
            .map_err(|e| eyre::eyre!("Failed to write to display: {}", e))?;

        Ok(ActionResponse {
            success: true,
            error: None,
        })
    }

    async fn write_color_buffer(
        &self,
        _buffer: Vec<u8>,
        _width: u32,
        _height: u32,
        _format: String,
        _brightness: u32,
    ) -> Result<ActionResponse> {
        Ok(ActionResponse {
            success: false,
            error: Some(Error {
                code: ErrorCode::NotImplemented as i32,
                message: "Color buffer not supported".to_string(),
            }),
        })
    }
}
