use super::feetech_servo::Sts3215;
use eyre::Result;
use std::collections::HashMap;
use std::os::raw::{c_int, c_short, c_uchar, c_uint, c_ushort};
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{info, trace, warn};
const MAX_SHMEM_DATA: usize = 2048;
pub const MAX_SERVOS: usize = 32;

#[repr(C)]
#[derive(Debug)]
pub struct ServoInfo {
    pub id: c_uchar,
    pub last_read_ms: c_uint,
    pub torque_switch: c_uchar,
    pub acceleration: c_uchar,
    pub target_location: c_short,
    pub running_time: c_ushort,
    pub running_speed: c_ushort,
    pub torque_limit: c_ushort,
    pub reserved1: [c_uchar; 6],
    pub lock_mark: c_uchar,
    pub current_location: c_short,
    pub current_speed: c_short,
    pub current_load: c_short,
    pub current_voltage: c_uchar,
    pub current_temperature: c_uchar,
    pub async_write_flag: c_uchar,
    pub servo_status: c_uchar,
    pub mobile_sign: c_uchar,
    pub reserved2: [c_uchar; 2],
    pub current_current: c_ushort,
}

#[repr(C)]
#[derive(Debug)]
pub struct ServoInfoBuffer {
    pub retry_count: c_uint,
    pub read_count: c_uint,
    pub loop_count: c_uint,
    pub fault_count: c_uint,
    pub last_read_ms: c_uint,
    pub servos: [ServoInfo; MAX_SERVOS],
}

#[repr(C)]
pub struct ActiveServoList {
    pub len: c_uint,
    pub servo_id: [c_uchar; MAX_SERVOS],
}

#[repr(C)]
pub struct ShmemData {
    pub data_length: c_uint,
    pub data: [c_uchar; MAX_SHMEM_DATA],
}

#[repr(C)]
#[derive(Debug)]
pub struct BroadcastCommand {
    pub data_length: c_uint,
    pub data: [c_uchar; MAX_SHMEM_DATA],
}

#[link(name = "feetech")]
#[allow(dead_code)]
extern "C" {
    fn servo_init() -> c_int;
    fn servo_deinit();
    fn servo_write(id: c_uchar, address: c_uchar, data: *const c_uchar, length: c_uchar) -> c_int;
    fn servo_read(id: c_uchar, address: c_uchar, data: *mut c_uchar, length: c_uchar) -> c_int;
    pub fn servo_set_active_servos(active_servos: ActiveServoList) -> c_int;
    pub fn servo_get_info(info_buffer: *mut ServoInfoBuffer) -> c_int;
    fn servo_broadcast_command(command: BroadcastCommand) -> c_int;
}

#[derive(Debug, Clone, Copy)]
pub enum FeetechOperationMode {
    PositionControl,
    SpeedControl,
    TorqueControl,
}

#[derive(Debug, Clone, Copy)]
pub enum FeetechActuatorType {
    Sts3215,
    Sts3250,
}

impl FeetechActuatorType {
    pub fn from_model_id(id: &[u8]) -> Option<Self> {
        if id.len() != 2 {
            return None;
        }
        match (id[0], id[1]) {
            (0x09, 0x03) => Some(FeetechActuatorType::Sts3215),
            (0x09, 0x11) => Some(FeetechActuatorType::Sts3250),
            _ => None,
        }
    }

    pub fn model_id(&self) -> [u8; 2] {
        match self {
            FeetechActuatorType::Sts3215 => [0x09, 0x03],
            FeetechActuatorType::Sts3250 => [0x09, 0x11],
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct FeetechActuatorInfo {
    pub id: u8,
    pub torque_enabled: bool,
    pub position_deg: f32,
    pub speed_deg_per_s: f32,
    pub load_percent: f32,
    pub voltage_v: f32,
    pub current_ma: f32,
    pub temperature_c: f32,
    pub last_read_ms: u32,
    pub last_read_local_ms: u32,
    pub online: bool,
    pub faults: Vec<String>,
}

pub trait FeetechActuator: Send + Sync + std::fmt::Debug {
    fn id(&self) -> u8;
    fn info(&self) -> FeetechActuatorInfo;
    fn set_position(&mut self, position_deg: f32) -> Result<()>;
    fn set_speed(&mut self, speed_deg_per_s: f32) -> Result<()>;
    fn set_acceleration(&mut self, speed_deg_per_s2: f32) -> Result<()>;
    fn enable_torque(&mut self) -> Result<()>;
    fn disable_torque(&mut self) -> Result<()>;
    fn change_id(&mut self, id: u8) -> Result<()>;
    fn update_info(&mut self, info: &ServoInfo);
    fn degrees_to_raw(&self, degrees: f32, offset: f32) -> u16;
    fn raw_to_degrees(&self, raw: u16, offset: f32) -> f32;
    fn set_pid(&mut self, p: Option<f32>, i: Option<f32>, d: Option<f32>) -> Result<()>;
    fn set_operation_mode(&mut self, mode: FeetechOperationMode) -> Result<()>;
    fn get_current(&self) -> Result<f32>;
    fn write_calibration_data(&mut self, min_angle: f32, max_angle: f32, offset: f32)
        -> Result<()>;
    fn set_zero_position(&mut self) -> Result<()>;
}

#[derive(Debug, Clone)]
pub struct FeetechSupervisor {
    pub servos: Arc<RwLock<HashMap<u8, Box<dyn FeetechActuator>>>>,
    pub actuator_desired_positions: HashMap<u8, f32>,
    //pub actuator_desired_time: HashMap<u8, f32>,
    pub actuator_desired_velocities: HashMap<u8, f32>
}

impl FeetechSupervisor {
    pub fn new() -> Result<Self> {
        unsafe {
            if servo_init() != 0 {
                return Err(eyre::eyre!("Failed to initialize servo system"));
            }
        }

        let supervisor = Self {
            servos: Arc::new(RwLock::new(HashMap::new())),
            actuator_desired_positions: HashMap::new(),
            //actuator_desired_time: HashMap::new(),
            actuator_desired_velocities: HashMap::new()
        };

        let supervisor_clone = supervisor.clone();

        tokio::spawn(async move {
            let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(8)); // 125hz
            let mut stats_interval = tokio::time::interval(tokio::time::Duration::from_secs(5)); // 5 seconds

            // Stats tracking
            let mut accumulated_stats = ServoInfoBuffer {
                retry_count: 0,
                read_count: 0,
                loop_count: 0,
                fault_count: 0,
                last_read_ms: 0,
                servos: unsafe { std::mem::zeroed() },
            };

            loop {
                tokio::select! {
                    _ = interval.tick() => {
                        let mut info_buffer = ServoInfoBuffer {
                            retry_count: 0,
                            read_count: 0,
                            loop_count: 0,
                            fault_count: 0,
                            last_read_ms: 0,
                            servos: unsafe { std::mem::zeroed() },
                        };

                        // Move info_buffer into the blocking task and get it back
                        let info_buffer = tokio::task::spawn_blocking(move || {
                            unsafe { servo_get_info(&mut info_buffer); }
                            info_buffer
                        })
                        .await
                        .unwrap();

                        // Accumulate stats
                        accumulated_stats.retry_count += info_buffer.retry_count;
                        accumulated_stats.read_count += info_buffer.read_count;
                        accumulated_stats.loop_count += info_buffer.loop_count;
                        accumulated_stats.fault_count += info_buffer.fault_count;

                        let mut servos = supervisor_clone.servos.write().await;
                        for servo in &info_buffer.servos {
                            if servo.id != 0 {
                                if let Some(actuator) = servos.get_mut(&servo.id) {
                                    actuator.update_info(servo);
                                }
                            }
                        }
                    }
                    _ = stats_interval.tick() => {
                        info!(
                            "Servo Stats (5s) - Retries: {}, Reads: {}, Loops: {}, Faults: {}",
                            accumulated_stats.retry_count,
                            accumulated_stats.read_count,
                            accumulated_stats.loop_count,
                            accumulated_stats.fault_count
                        );

                        // Reset accumulated stats
                        accumulated_stats.retry_count = 0;
                        accumulated_stats.read_count = 0;
                        accumulated_stats.loop_count = 0;
                        accumulated_stats.fault_count = 0;
                    }
                }
            }
        });

        Ok(supervisor)
    }

    pub async fn update_active_servos(&mut self) -> Result<()> {
        let servos = self.servos.write().await;
        let servo_ids = servos.iter().map(|(id, _)| *id).collect::<Vec<_>>();

        // Create ActiveServoList with proper initialization
        let mut active_servos = ActiveServoList {
            len: servo_ids.len() as c_uint,
            servo_id: [0; MAX_SERVOS],
        };

        // Copy the IDs into the fixed-size array
        for (i, &id) in servo_ids.iter().enumerate() {
            if i >= MAX_SERVOS {
                break;
            }
            active_servos.servo_id[i] = id as c_uchar;
        }

        unsafe {
            if servo_set_active_servos(active_servos) != 0 {
                return Err(eyre::eyre!("Failed to set active servos"));
            }
        }
        Ok(())
    }

    pub async fn add_servo(&mut self, id: u8, actuator_type: FeetechActuatorType) -> Result<()> {
        let mut servos = self.servos.write().await;
        let mut actuator = match actuator_type {
            FeetechActuatorType::Sts3215 => Sts3215::new(id),
            FeetechActuatorType::Sts3250 => todo!(),
        };
        let mut success = false;
        for _ in 0..10 {
            if actuator.check_id().is_ok() {
                success = true;
                break;
            }
        }

        if success {
            servos.insert(id, Box::new(actuator));
            drop(servos);
            self.update_active_servos().await?;
        } else {
            warn!(
                "Failed to add servo {:?} not responding after 10 attempts",
                id
            );
        }
        Ok(())
    }

    pub async fn remove_servo(&mut self, id: u8) -> Result<()> {
        let mut servos = self.servos.write().await;
        servos.remove(&id);
        drop(servos);
        self.update_active_servos().await?;
        Ok(())
    }

    //pub async fn move_actuators(&mut self, desired_positions: &HashMap<u8, f32>, desired_time: &HashMap<u8, f32>, desired_velocities: &HashMap<u8, f32>) -> Result<()> {
    pub async fn move_actuators(&mut self, desired_positions: &HashMap<u8, f32>, desired_velocities: &HashMap<u8, f32>) -> Result<()> {
        for (id, position) in desired_positions {
            self.actuator_desired_positions.insert(*id, *position);
        }

        /*for (id, time) in desired_time {
            self.actuator_desired_time.insert(*id, *time);
        }*/

        for (id, velocity) in desired_velocities {
            self.actuator_desired_velocities.insert(*id, *velocity);
        }

        self.broadcast_command().await?;

        Ok(())
    }

    pub async fn disable_torque(&mut self, id: u8) -> Result<()> {
        {
            // New scope to ensure servos lock is dropped
            let mut servos = self.servos.write().await;
            let servo = servos
                .get_mut(&id)
                .ok_or_else(|| eyre::eyre!("Servo with id {} not found", id))?;
            servo.disable_torque()?;
        } // servos lock is dropped here
        self.broadcast_command().await?;
        Ok(())
    }

    pub async fn enable_torque(&mut self, id: u8) -> Result<()> {
        {
            // New scope to ensure servos lock is dropped
            let mut servos = self.servos.write().await;
            let servo = servos
                .get_mut(&id)
                .ok_or_else(|| eyre::eyre!("Servo with id {} not found", id))?;
            servo.enable_torque()?;
            self.actuator_desired_positions.remove(&id);
            //self.actuator_desired_time.remove(&id);
            self.actuator_desired_velocities.remove(&id);
        } // servos lock is dropped here
        self.broadcast_command().await?;
        Ok(())
    }

    pub async fn broadcast_command(&mut self) -> Result<()> {
        let mut command = BroadcastCommand {
            data_length: 0,
            data: [0; MAX_SHMEM_DATA],
        };
    
        const SERVO_ADDR_TARGET_POSITION: u8 = 0x2A; // Address for position & velocity
        let mut index = 0;
    
        let servos = self.servos.read().await;
    
        // Write header
        command.data[index] = SERVO_ADDR_TARGET_POSITION;
        index += 1;
        command.data[index] = 6; // Each servo entry takes 6 bytes (Position + Time + Speed)
        index += 1;
    
        for (id, position) in &self.actuator_desired_positions {
            if let Some(servo) = servos.get(id) {
                if servo.info().torque_enabled {
                    let position_raw = servo.degrees_to_raw(*position, 180.0);
                    /*let time = self.actuator_desired_time.get(id).copied().unwrap_or(0.0);
                    let time_raw: u16 = (time * 1000.0).clamp(0.0, u16::MAX as f32) as u16;
                    let velocity = if time > 0.0 {
                        0.0 // Set velocity to zero if time is greater than 0
                    } else {
                        self.actuator_desired_velocities.get(id).copied().unwrap_or(1000.0) // 1000 larger than max
                    };*/

                    let velocity = self.actuator_desired_velocities.get(id).copied().unwrap_or(1000.0); // 1000 larger than max
                    let velocity_raw = servo.degrees_to_raw(velocity, 0.0);
                    
                    // Pack data
                    command.data[index] = *id;
                    index += 1;
                    command.data[index] = (position_raw & 0xFF) as u8; // Position LSB
                    index += 1;
                    command.data[index] = ((position_raw >> 8) & 0xFF) as u8; // Position MSB
                    index += 1;
                    /*command.data[index] = (time_raw & 0xFF) as u8; // Time LSB
                    index += 1;
                    command.data[index] =((time_raw >> 8) & 0xFF) as u8;// Time MSB
                    index += 1;*/
                    command.data[index] = 0x00; // Time LSB
                    index += 1;
                    command.data[index] = 0x00; // Time MSB
                    index += 1;
                    command.data[index] = (velocity_raw & 0xFF) as u8; // Velocity LSB
                    index += 1;
                    command.data[index] = ((velocity_raw >> 8) & 0xFF) as u8; // Velocity MSB
                    index += 1;
                }
            }
        }
    
        command.data_length = index as c_uint;
    
        unsafe {
            if servo_broadcast_command(command) != 0 {
                return Err(eyre::eyre!("Failed to broadcast command"));
            }
        }
    
        Ok(())
    }

    pub async fn change_id(&mut self, id: u8, new_id: u8) -> Result<()> {
        let mut servos = self.servos.write().await;
        if let Some(servo) = servos.get_mut(&id) {
            servo.change_id(new_id)?;
            Ok(())
        } else {
            let mut new_servo = Box::new(Sts3215::new(id));
            new_servo.change_id(new_id)?;
            Ok(())
        }
    }
}

impl Drop for FeetechSupervisor {
    fn drop(&mut self) {
        unsafe {
            servo_deinit();
        }
    }
}

pub fn feetech_write(id: u8, address: u8, data: &[u8]) -> Result<()> {
    trace!(
        "feetech_write: id: {}, address: {}, data: {:?}",
        id,
        address,
        data
    );
    let mut attempts = 0;
    const MAX_ATTEMPTS: u8 = 15;

    loop {
        let result = unsafe { servo_write(id, address, data.as_ptr(), data.len() as u8) };

        if result == 0 {
            break;
        }

        attempts += 1;
        trace!(
            "Write failed (attempt {}/{}): servo id: {}, address: {}, result: {}",
            attempts,
            MAX_ATTEMPTS,
            id,
            address,
            result
        );

        if attempts >= MAX_ATTEMPTS {
            return Err(eyre::eyre!(
                "Failed to write to servo after {} attempts, last result: {}",
                MAX_ATTEMPTS,
                result
            ));
        }

        std::thread::sleep(std::time::Duration::from_nanos(200));
    }
    Ok(())
}

pub fn feetech_read(id: u8, address: u8, length: u8) -> Result<Vec<u8>> {
    let mut data = vec![0u8; length as usize];
    let mut attempts = 0;
    const MAX_ATTEMPTS: u8 = 15;

    trace!(
        "feetech_read: id: {}, address: {}, length: {}",
        id,
        address,
        length
    );

    loop {
        let result = unsafe { servo_read(id, address, data.as_mut_ptr(), length) };

        if result == 0 {
            break;
        }

        attempts += 1;
        trace!(
            "Read failed (attempt {}/{}): servo id: {}, address: {}, result: {}",
            attempts,
            MAX_ATTEMPTS,
            id,
            address,
            result
        );

        if attempts >= MAX_ATTEMPTS {
            return Err(eyre::eyre!(
                "Failed to read from servo after {} attempts, last result: {}",
                MAX_ATTEMPTS,
                result
            ));
        }

        std::thread::sleep(std::time::Duration::from_nanos(200));
    }
    Ok(data)
}

pub fn feetech_init() -> Result<()> {
    unsafe {
        if servo_init() != 0 {
            return Err(eyre::eyre!("Failed to initialize servo system"));
        }
    }
    Ok(())
}

pub fn feetech_deinit() -> Result<()> {
    unsafe {
        servo_deinit();
    }
    Ok(())
}
