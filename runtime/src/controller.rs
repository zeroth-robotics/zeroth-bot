use anyhow::Result;
use runtime::hal::{Servo, ServoMultipleWriteCommand, MAX_SERVOS};
use tokio::time::{sleep, interval, Duration};
use std::sync::Arc;
use tokio::sync::Mutex;
use crate::model::Model;
use ndarray::Array1;

pub struct Robot{

    servo: Servo,
    cycle_time: f32,
    prev_actions: [f32; 10],
    prev_buffer: [f32; 615],
}

impl Robot{
    pub fn new() -> Result<Self> {
        let servo = Servo::new()?;

        Ok(Self { servo, cycle_time: 0.5, prev_actions: [0.0; 10], prev_buffer: [0.0; 615] })
    }

    pub async fn run(&mut self, model: Arc<Model>) -> Result<()> {
        let mut control_interval = interval(Duration::from_millis(20));

        loop {
            control_interval.tick().await;

            // get joint states
            let model_input = self.get_robot_state().await?;
            
            // get desired joint positions (inferenced from model)
            let desired_joint_positions = self.model_inference(&model, &model_input).await?;
            
            // send joint commands
            self.send_joint_commands(&desired_joint_positions).await?;
        }
    }

    async fn get_robot_state(&self) -> Result<[f32; 656]> {
        let servo_data = self.servo.read_continuous()?;
        // first 3 elements are x_vel, y_vel, rot, (set to 0)
        // t is current time
        // dof_pos is current joint positions
        // dof_vel is current joint velocities
        // prev_actions is previous actions
        // imu_ang_vel is angular velocity of the IMU
        // imu_euler_xyz is euler angles of the IMU
        let mut combined_robot_state = [0.0; 656]; // 41 + 615

        let time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f32();
        
        let sin_time = (2.0 * std::f32::consts::PI * time / self.cycle_time).sin();
        let cos_time = (2.0 * std::f32::consts::PI * time / self.cycle_time).cos();

        combined_robot_state[3] = sin_time;
        combined_robot_state[4] = cos_time;

        let joint_positions: [f32; 10] = servo_data.servo.iter()
            .take(10)
            .map(|s| s.current_location as f32)
            .collect::<Vec<f32>>()
            .try_into()
            .unwrap_or([0.0; 10]);

        let joint_velocities: [f32; 10] = servo_data.servo.iter()
            .take(10)
            .map(|s| s.current_speed as f32)
            .collect::<Vec<f32>>()
            .try_into()
            .unwrap_or([0.0; 10]);

        combined_robot_state[5..15].copy_from_slice(&joint_positions);
        combined_robot_state[15..25].copy_from_slice(&joint_velocities);
        combined_robot_state[25..35].copy_from_slice(&self.prev_actions);

        let imu_ang_vel = [0.0; 3]; // TOOD: IMU
        let imu_euler_xyz = [0.0; 3];

        combined_robot_state[35..38].copy_from_slice(&imu_ang_vel);
        combined_robot_state[38..41].copy_from_slice(&imu_euler_xyz);

        combined_robot_state[41..656].copy_from_slice(&self.prev_buffer);

        Ok(combined_robot_state)
    }

    async fn model_inference(&mut self, model: &Model, model_input: &[f32; 656]) -> Result<[f32; 16]> {

        // x_vel: Array1<f32>,
        // y_vel: Array1<f32>,
        // rot: Array1<f32>,
        // t_sin: Array1<f32>,
        // t_cos: Array1<f32>, 
        // dof_pos: Array1<f32>,
        // dof_vel: Array1<f32>,
        // prev_actions: Array1<f32>,
        // imu_ang_vel: Array1<f32>,
        // imu_euler_xyz: Array1<f32>,
        // hist_obs: Array1<f32>,

        let model_output = model.infer(model_input)?;
        // model output is {"actions": actions_scaled, "actions_raw": actions, "new_x": x}
        // 10 dim each for actions, 615 dim for new_x

        self.prev_actions.copy_from_slice(&model_output[..10]);  // Store actions
        self.prev_buffer.copy_from_slice(&model_output[20..635]);

        let mut desired_joint_positions = [0.0; 16];  // Initialize all positions to 0.0
        desired_joint_positions[..10].copy_from_slice(&model_output[..10]);

        Ok(desired_joint_positions)   
    }

    async fn send_joint_commands(&self, positions: &[f32; 16]) -> Result<()> {
        let mut cmd = ServoMultipleWriteCommand {
            ids: [0; MAX_SERVOS],
            positions: [0; MAX_SERVOS],
            times: [0; MAX_SERVOS],
            speeds: [0; MAX_SERVOS],
            only_write_positions: 0,
        };

        for i in 0..16 {
            cmd.ids[i] = (i + 1) as u8;
            cmd.positions[i] = positions[i] as i16;
            cmd.times[i] = 20;
        }

        self.servo.write_multiple(&cmd)?;

        println!("Command sent to move all servos to position {} with time {} ms and speed {}, send_only_positions: {}", positions[0] as i16, 20, 0, 0);
        Ok(())
    }
}

#[tokio::main]
pub async fn run(model: Arc<Model>, robot: Arc<Mutex<Robot>>) -> Result<()> {
    let mut robot = robot.lock().await;
    robot.servo.enable_readout()?;  
    robot.run(model).await?;

    Ok(())
}



