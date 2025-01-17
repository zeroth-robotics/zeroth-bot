use kos_zbot::feetech::{feetech_deinit, feetech_init, FeetechActuator};
use kos_zbot::feetech_servo::Sts3215;
use std::env;
use std::thread;
use std::time::Duration;

fn main() {
    // Get command line arguments
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        println!("Usage: {} <old_id> <new_id>", args[0]);
        return;
    }

    let old_id: u8 = args[1]
        .parse()
        .expect("Old ID must be a number between 1-255");
    let new_id: u8 = args[2]
        .parse()
        .expect("New ID must be a number between 1-255");

    if !(1..=255).contains(&old_id) || !(1..=255).contains(&new_id) {
        println!("IDs must be between 1 and 255");
        return;
    }

    feetech_init().unwrap();

    // Check if old ID exists
    let mut servo = Sts3215::new(old_id);
    if servo.check_id().is_err() {
        println!("No servo found at ID {}", old_id);
        feetech_deinit().unwrap();
        return;
    }

    // Check if new ID is already taken
    let mut test_servo = Sts3215::new(new_id);
    if test_servo.check_id().is_ok() {
        println!("ID {} is already in use", new_id);
        feetech_deinit().unwrap();
        return;
    }

    // Change ID
    println!("Changing ID from {} to {}...", old_id, new_id);
    match servo.change_id(new_id) {
        Ok(()) => {
            // Verify the change with retries
            let mut success = false;
            for attempt in 1..=5 {
                thread::sleep(Duration::from_millis(10)); // Add 10ms delay between attempts
                let mut new_servo = Sts3215::new(new_id);
                match new_servo.check_id() {
                    Ok(()) => {
                        println!("Successfully changed ID to {}", new_id);
                        success = true;
                        break;
                    }
                    Err(_) => {
                        if attempt < 5 {
                            println!("Verification attempt {} failed, retrying...", attempt);
                        } else {
                            println!("Failed to verify new ID {} after 5 attempts", new_id);
                        }
                    }
                }
            }
            if !success {
                println!("Warning: ID change might have failed");
            }
        }
        Err(e) => println!("Failed to change ID: {}", e),
    }

    feetech_deinit().unwrap();
}
