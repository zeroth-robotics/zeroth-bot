use kos_zbot::feetech_servo::Sts3215;
use kos_zbot::feetech::{feetech_init, feetech_deinit};
use std::env;

fn main() {
    feetech_init().unwrap();

    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        println!("Usage: {} <id>", args[0]);
        return;
    }

    let id: u8 = args[1].parse().expect("ID must be a number between 1-255");

    let mut servo = Sts3215::new(id);
    if servo.check_id().is_ok() {

    } else {
        println!("No servo found at ID {}", id);
    }

    feetech_deinit().unwrap();
}