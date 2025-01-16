use kos_zbot::feetech_servo::Sts3215;
use kos_zbot::feetech::{feetech_init, feetech_deinit, FeetechActuator, FeetechOperationMode};
use std::env;
use std::thread::sleep;
use std::time::Duration;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        println!("Usage: {} <id>", args[0]);
        return;
    }

    let id: u8 = args[1].parse().expect("ID must be a number between 1-255");
    println!("Initializing Feetech bus...");
    feetech_init().unwrap();

    let mut servo = Sts3215::new(id);
    if servo.check_id().is_ok() {
        println!("Found servo at ID {}", id);
        println!("Testing servo movement...");
        
        println!("Enabling torque...");
        servo.enable_torque().unwrap();
        
        println!("Setting to speed control mode...");
        servo.set_operation_mode(FeetechOperationMode::SpeedControl).unwrap();
        
        println!("Running oscillation test (100 cycles)...");
        for i in 0..10 {
            if i % 10 == 0 {
                println!("Cycle {}/100", i);
            }
            servo.set_speed(20.0).unwrap();
            sleep(Duration::from_millis(200));
            servo.set_speed(-20.0).unwrap();
            sleep(Duration::from_millis(200));
        }
        
        println!("Stopping servo...");
        for _ in 0..10 {
            servo.set_speed(0.0).unwrap();
            sleep(Duration::from_millis(10));
        }
        
        println!("Returning to position control mode...");
        servo.set_operation_mode(FeetechOperationMode::PositionControl).unwrap();
        
        println!("Disabling torque...");
        servo.disable_torque().unwrap();
        
        println!("Test complete!");
    } else {
        println!("No servo found at ID {}", id);
    }

    println!("Deinitializing Feetech bus...");
    feetech_deinit().unwrap();
}