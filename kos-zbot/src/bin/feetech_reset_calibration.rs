use kos_zbot::feetech::{feetech_deinit, feetech_init, FeetechActuator};
use kos_zbot::feetech_servo::Sts3215;
use std::env;
use tracing_subscriber::filter::EnvFilter;
use tracing_subscriber::prelude::*;
use tracing_subscriber::Layer;

fn main() {

    let subscriber = tracing_subscriber::registry();

    let stdout_layer = tracing_subscriber::fmt::layer()
        .with_writer(std::io::stdout)
        .with_filter(
            EnvFilter::from_default_env()
                .add_directive("h2=error".parse().unwrap())
                .add_directive("grpc=error".parse().unwrap())
                .add_directive("rumqttc=error".parse().unwrap())
                .add_directive("kos::telemetry=error".parse().unwrap())
                .add_directive("polling=error".parse().unwrap())
                .add_directive("async_io=error".parse().unwrap())
                .add_directive("krec=error".parse().unwrap()),
        );

    let _subscriber = subscriber.with(stdout_layer);

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
