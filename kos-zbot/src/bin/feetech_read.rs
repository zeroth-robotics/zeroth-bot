use kos_zbot::feetech::{
    feetech_deinit, feetech_init, servo_get_info, servo_set_active_servos, ActiveServoList,
    FeetechActuator, ServoInfoBuffer, MAX_SERVOS,
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
        loop {
            unsafe {
                servo_get_info(&mut info_buffer);
            }

            servo.update_info(&info_buffer.servos[0]);
            let pos = servo.info.position_deg;
            println!(
                "Position: {}, raw: {}",
                pos,
                servo.degrees_to_raw(pos, 180.0)
            );
            sleep(Duration::from_millis(10));
        }
    }

    feetech_deinit().unwrap();
}
