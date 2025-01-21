use kos_zbot::feetech::{feetech_deinit, feetech_init, feetech_read, feetech_write, FeetechActuatorType};
use kos_zbot::feetech_servo::Sts3215;

fn main() {
    println!("Scanning for Feetech servos...");
    feetech_init().unwrap();

    for id in 1..=255 {
        let mut servo = Sts3215::new(id);
        if servo.check_id().is_ok() {
            let model = feetech_read(id, 0x03, 2).unwrap();
            let min_angle = u16::from_le_bytes(feetech_read(id, 0x09, 2).unwrap().try_into().unwrap());
            let max_angle = u16::from_le_bytes(feetech_read(id, 0x0B, 2).unwrap().try_into().unwrap());
            let mode = feetech_read(id, 0x21, 1).unwrap();
            let status = feetech_read(id, 65, 1).unwrap();
            let status_str = decode_status(status[0]);

            let voltage = feetech_read(id, 0x3E, 1).unwrap();
            let temperature = feetech_read(id, 0x3F, 1).unwrap();
            let actuator_type = FeetechActuatorType::from_model_id(&model);

            let max_temp_limit = feetech_read(id, 13, 1).unwrap();
            let max_voltage_limit = feetech_read(id, 14, 1).unwrap();
            let min_voltage_limit = feetech_read(id, 15, 1).unwrap();


            if max_temp_limit[0] == 0 {
                feetech_write(id, 0x37, &[0x00]).unwrap();
                feetech_write(id, 13, &[70]).unwrap();
                feetech_write(id, 0x37, &[0x01]).unwrap();


            }

            if max_voltage_limit[0] == 0 {
                feetech_write(id, 0x37, &[0x00]).unwrap();
                feetech_write(id, 14, &[140]).unwrap();
                feetech_write(id, 0x37, &[0x01]).unwrap();
            }

            if let Some(actuator_type) = actuator_type {
                println!("{:3} Model: {:?}, Min: {:?}, Max: {:?}, Mode: {:?}, Status: {} ({:?}), Voltage: {:?}, Temperature: {:?}, Max Temp Limit: {:?}, Max Voltage Limit: {:?}, Min Voltage Limit: {:?}", 
                    id, actuator_type, min_angle, max_angle, mode, status_str, status[0], voltage[0] as f32 / 10.0, temperature[0], max_temp_limit[0], max_voltage_limit[0] as f32 / 10.0, min_voltage_limit[0] as f32 / 10.0);
            } else {
                println!("{:3} Unknown model: {:?}", id, model);
            }
        }
    }

    feetech_deinit().unwrap();
}

fn decode_status(status: u8) -> String {
    let mut errors = Vec::new();
    
    if status & (1 << 0) != 0 { errors.push("Voltage"); }
    if status & (1 << 1) != 0 { errors.push("Sensor"); }
    if status & (1 << 2) != 0 { errors.push("Temperature"); }
    if status & (1 << 3) != 0 { errors.push("Current"); }
    if status & (1 << 4) != 0 { errors.push("Angle"); }
    if status & (1 << 5) != 0 { errors.push("Overload"); }

    if errors.is_empty() {
        "OK".to_string()
    } else {
        errors.join(", ")
    }
}
