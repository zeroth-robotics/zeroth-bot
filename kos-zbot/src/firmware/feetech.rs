use super::feetech_servo::Sts3215;
use eyre::Result;
use std::collections::HashMap;
use std::os::raw::{c_int, c_short, c_uchar, c_uint, c_ushort};
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::info;
const MAX_SHMEM_DATA: usize = 2048;
const MAX_SERVOS: usize = 32;

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
    fn servo_read(id: c_uchar, address: c_uchar, length: c_uchar, data: *mut c_uchar) -> c_int;
    fn servo_set_active_servos(active_servos: ActiveServoList) -> c_int;
    fn servo_get_info(info_buffer: *mut ServoInfoBuffer) -> c_int;
    fn servo_broadcast_command(command: BroadcastCommand) -> c_int;
}

#[derive(Debug, Clone, Copy)]
pub enum FeetechActuatorType {
    Sts3215,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct FeetechActuatorInfo {
    pub id: u8,
    pub torque_enabled: bool,
    pub position_deg: f32,
    pub speed_deg_per_s: f32,
    pub load_percent: f32,
    pub voltage_v: f32,
    pub current_ma: f32,
    pub temperature_c: f32,
}

pub trait FeetechActuator: Send + Sync + std::fmt::Debug {
    fn id(&self) -> u8;
    fn info(&self) -> FeetechActuatorInfo;
    fn set_position(&mut self, position_deg: f32) -> Result<()>;
    fn set_speed(&mut self, speed_deg_per_s: f32) -> Result<()>;
    fn enable_torque(&mut self) -> Result<()>;
    fn disable_torque(&mut self) -> Result<()>;
    fn change_id(&mut self, id: u8) -> Result<()>;
    fn update_info(&mut self, info: &ServoInfo);
    fn degrees_to_raw(&self, degrees: f32) -> u16;
    fn raw_to_degrees(&self, raw: u16) -> f32;
    fn set_pid(&mut self, p: Option<f32>, i: Option<f32>, d: Option<f32>) -> Result<()>;
}

#[derive(Debug, Clone)]
pub struct FeetechSupervisor {
    pub servos: Arc<RwLock<HashMap<u8, Box<dyn FeetechActuator>>>>,
    pub actuator_desired_positions: HashMap<u8, f32>,
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
        };

        let supervisor_clone = supervisor.clone();

        let mut info_buffer = ServoInfoBuffer {
            retry_count: 0,
            read_count: 0,
            loop_count: 0,
            fault_count: 0,
            last_read_ms: 0,
            servos: unsafe { std::mem::zeroed() },
        };

        tokio::spawn(async move {
            let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(5)); // 200hz
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
                        unsafe {
                            servo_get_info(&mut info_buffer);
                        }

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
        let actuator = match actuator_type {
            FeetechActuatorType::Sts3215 => Sts3215::new(id),
        };
        servos.insert(id, Box::new(actuator));
        drop(servos);
        self.update_active_servos().await?;
        Ok(())
    }

    pub async fn remove_servo(&mut self, id: u8) -> Result<()> {
        let mut servos = self.servos.write().await;
        servos.remove(&id);
        drop(servos);
        self.update_active_servos().await?;
        Ok(())
    }

    pub async fn move_actuators(&mut self, desired_positions: &HashMap<u8, f32>) -> Result<()> {
        for (id, position) in desired_positions {
            self.actuator_desired_positions.insert(*id, *position);
        }

        self.broadcast_command().await?;

        Ok(())
    }

    pub async fn disable_torque(&mut self, id: u8) -> Result<()> {
        {
            // New scope to ensure servos lock is dropped
            let mut servos = self.servos.write().await;
            servos.get_mut(&id).unwrap().disable_torque()?;
        } // servos lock is dropped here
        self.broadcast_command().await?;
        Ok(())
    }

    pub async fn enable_torque(&mut self, id: u8) -> Result<()> {
        {
            // New scope to ensure servos lock is dropped
            let mut servos = self.servos.write().await;
            servos.get_mut(&id).unwrap().enable_torque()?;
            self.actuator_desired_positions.remove(&id);
        } // servos lock is dropped here
        self.broadcast_command().await?;
        Ok(())
    }

    pub async fn broadcast_command(&mut self) -> Result<()> {
        let mut command = BroadcastCommand {
            data_length: 0,
            data: [0; MAX_SHMEM_DATA],
        };

        const SERVO_ADDR_TARGET_POSITION: u8 = 0x2A;
        let mut index = 0;

        // Write header
        command.data[index] = SERVO_ADDR_TARGET_POSITION;
        index += 1;
        command.data[index] = 2; // Data length per servo (2 for position only)
        index += 1;

        let servos = self.servos.read().await;

        // Write data for each servo
        for (id, position) in &self.actuator_desired_positions {
            if let Some(servo) = servos.get(id) {
                if servo.info().torque_enabled {
                    let position_raw = servo.degrees_to_raw(*position);

                    command.data[index] = *id;
                    index += 1;
                    command.data[index] = (position_raw & 0xFF) as u8;
                    index += 1;
                    command.data[index] = ((position_raw >> 8) & 0xFF) as u8;
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
    unsafe {
        if servo_write(id, address, data.as_ptr(), data.len() as u8) != 0 {
            return Err(eyre::eyre!("Failed to write to servo"));
        }
    }
    Ok(())
}
