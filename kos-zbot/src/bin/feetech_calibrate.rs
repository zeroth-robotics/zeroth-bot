use kos_zbot::feetech::{
    feetech_deinit, feetech_init, servo_get_info, servo_set_active_servos, ActiveServoList,
    FeetechActuator, FeetechOperationMode, ServoInfoBuffer, MAX_SERVOS,
};
use kos_zbot::feetech_servo::Sts3215;
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
    feetech_init().unwrap();

    let mut servo = Sts3215::new(id);

    let mut active_servos = ActiveServoList {
        len: 1,
        servo_id: [0; MAX_SERVOS],
    };
    active_servos.servo_id[0] = id;

    unsafe {
        servo_set_active_servos(active_servos);
    }

    let mut info_buffer = ServoInfoBuffer {
        retry_count: 0,
        read_count: 0,
        loop_count: 0,
        fault_count: 0,
        last_read_ms: 0,
        servos: unsafe { std::mem::zeroed() },
    };

    if servo.check_id().is_ok() {
        servo.enable_torque().unwrap();
        sleep(Duration::from_millis(100));

        // First direction (negative)
        println!("Moving to first end stop...");
        for _ in 0..10 {
            servo
                .set_operation_mode(FeetechOperationMode::SpeedControl)
                .unwrap();
            servo.set_speed(-15.0).unwrap();
            sleep(Duration::from_millis(20));
        }

        // Wait for initial current spike to pass
        sleep(Duration::from_millis(500));

        while servo.get_current().unwrap_or(0.0) < 1000.0 {
            sleep(Duration::from_millis(20));
        }

        unsafe {
            servo_get_info(&mut info_buffer);
        }
        servo.update_info(&info_buffer.servos[0]);
        let min_position = servo.info.position_deg;
        println!("Min position: {}", min_position);

        // Stop and wait
        servo.set_speed(0.0).unwrap();
        sleep(Duration::from_millis(500));

        // Second direction (positive)
        println!("Moving to second end stop...");
        for _ in 0..10 {
            servo
                .set_operation_mode(FeetechOperationMode::SpeedControl)
                .unwrap();
            servo.set_speed(15.0).unwrap();
            sleep(Duration::from_millis(20));
        }

        // Wait for initial current spike to pass
        sleep(Duration::from_millis(500));

        while servo.get_current().unwrap_or(0.0) < 1000.0 {
            sleep(Duration::from_millis(20));
        }

        unsafe {
            servo_get_info(&mut info_buffer);
        }
        servo.update_info(&info_buffer.servos[0]);
        let max_position = servo.info.position_deg;
        println!("Max position: {}", max_position);

        servo.set_speed(0.0).unwrap();

        println!(
            "Range: {} to {} (total: {})",
            min_position,
            max_position,
            max_position - min_position
        );

        servo
            .write_calibration_data(min_position, max_position, 0.0)
            .unwrap();

        servo
            .set_operation_mode(FeetechOperationMode::PositionControl)
            .unwrap();

        println!("Disabling torque...");
        servo.disable_torque().unwrap();

        println!("Test complete!");
    } else {
        println!("No servo found at ID {}", id);
    }

    feetech_deinit().unwrap();
}
