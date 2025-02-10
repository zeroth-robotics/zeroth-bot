use kos_zbot::feetech::{feetech_deinit, feetech_init, FeetechActuator};
use kos_zbot::feetech_servo::Sts3215;
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        println!("Usage: {} <id> <deg/s^2>", args[0]);
        return;
    }
    let id: u8 = args[1].parse().expect("ID must be a number between 1-255");
    let acceleration: f32 = args[2].parse().expect("ID must be a number between 0-2230 deg/sec^2");

    feetech_init().unwrap();
    let mut servo = Sts3215::new(id);
    if servo.check_id().is_ok() {
        println!("found servo at [id]:{}", id);
        println!("setting acceleration {} deg/s^2", acceleration);
        servo.set_acceleration(acceleration).unwrap();
    } else {
        println!("no servo found at [id]:{}", id);
    }
    feetech_deinit().unwrap();
}