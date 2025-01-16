use kos_zbot::feetech::{feetech_deinit, feetech_init, feetech_read, FeetechActuatorType};
use kos_zbot::feetech_servo::Sts3215;

fn main() {
    println!("Scanning for Feetech servos...");
    feetech_init().unwrap();

    for id in 1..=255 {
        let mut servo = Sts3215::new(id);
        if servo.check_id().is_ok() {
            let model = feetech_read(id, 0x03, 2).unwrap();
            let actuator_type = FeetechActuatorType::from_model_id(&model);

            if let Some(actuator_type) = actuator_type {
                println!("{:3} Model: {:?}", id, actuator_type);
            } else {
                println!("{:3} Unknown model: {:?}", id, model);
            }
        }
    }

    feetech_deinit().unwrap();
}
