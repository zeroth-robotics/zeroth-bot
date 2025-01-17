use kos_zbot::feetech::{feetech_deinit, feetech_init, FeetechActuator};
use kos_zbot::feetech_servo::Sts3215;
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        println!("Usage: {} <id>", args[0]);
        return;
    }

    let id: u8 = args[1].parse().expect("ID must be a number between 1-255");
    feetech_init().unwrap();

    let mut servo = Sts3215::new(id);

    if servo.check_id().is_ok() {
        servo.write_calibration_data(-180.0, 180.0, 0.0).unwrap();
    } else {
        println!("No servo found at ID {}", id);
    }

    feetech_deinit().unwrap();
}
