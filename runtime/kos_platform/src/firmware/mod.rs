mod feetech;

pub use feetech::Servo;

pub mod hal {
    use serde::{Deserialize, Serialize};
    use std::os::raw::{c_short, c_uchar, c_uint, c_ushort};

    pub const MAX_SERVOS: usize = 16;

    #[repr(C)]
    #[derive(Debug, Copy, Clone, Default, Serialize, Deserialize)]
    pub struct ServoInfo {
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

    #[repr(u8)]
    #[derive(Debug, Copy, Clone, Serialize, Deserialize)]
    pub enum ServoRegister {
        FirmwareMajorVersion = 0x00,
        FirmwareSubVersion = 0x01,
        ServoMainVersion = 0x03,
        ServoSubVersion = 0x04,
        ID = 0x05,
        BaudRate = 0x06,
        ReturnDelay = 0x07,
        ResponseStatusLevel = 0x08,
        MinAngleLimit = 0x09,
        MaxAngleLimit = 0x0B,
        MaxTemperatureLimit = 0x0D,
        MaxInputVoltage = 0x0E,
        MinInputVoltage = 0x0F,
        MaxTorque = 0x10,
        Phase = 0x12,
        UnloadingCondition = 0x13,
        LEDAlarmCondition = 0x14,
        PProportionalCoeff = 0x15,
        DDifferentialCoeff = 0x16,
        IIntegralCoeff = 0x17,
        MinStartupForce = 0x18,
        ClockwiseInsensitiveArea = 0x1A,
        CounterclockwiseInsensitiveArea = 0x1B,
        ProtectionCurrent = 0x1C,
        AngularResolution = 0x1E,
        PositionCorrection = 0x1F,
        OperationMode = 0x21,
        ProtectiveTorque = 0x22,
        ProtectionTime = 0x23,
        OverloadTorque = 0x24,
        SpeedClosedLoopPCoeff = 0x25,
        OverCurrentProtectionTime = 0x26,
        VelocityClosedLoopICoeff = 0x27,
        TorqueSwitch = 0x28,
        Acceleration = 0x29,
        TargetLocation = 0x2A,
        RunningTime = 0x2C,
        RunningSpeed = 0x2E,
        TorqueLimit = 0x30,
        LockMark = 0x37,
        CurrentLocation = 0x38,
        CurrentSpeed = 0x3A,
        CurrentLoad = 0x3C,
        CurrentVoltage = 0x3E,
        CurrentTemperature = 0x3F,
        AsyncWriteFlag = 0x40,
        ServoStatus = 0x41,
        MobileSign = 0x42,
        CurrentCurrent = 0x45,
    }

    #[repr(u8)]
    #[derive(Debug, Copy, Clone, Serialize, Deserialize)]
    pub enum MemoryLockState {
        Unlocked = 0,
        Locked = 1,
    }

    #[repr(C)]
    #[derive(Debug, Copy, Clone, Serialize, Deserialize)]
    pub struct ServoData {
        pub servo: [ServoInfo; MAX_SERVOS],
        pub task_run_count: c_uint,
    }

    #[repr(C)]
    #[derive(Debug, Copy, Clone, Serialize, Deserialize)]
    pub struct ServoMultipleWriteCommand {
        pub only_write_positions: c_uchar,
        pub ids: [c_uchar; MAX_SERVOS],
        pub positions: [c_short; MAX_SERVOS],
        pub times: [c_ushort; MAX_SERVOS],
        pub speeds: [c_ushort; MAX_SERVOS],
    }

    #[repr(u8)]
    #[derive(Debug, Copy, Clone, Serialize, Deserialize)]
    pub enum ServoMode {
        Position = 0,
        ConstantSpeed = 1,
        PWMOpenLoop = 2,
        StepServo = 3,
    }

    #[repr(i32)]
    #[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
    pub enum ServoDirection {
        Clockwise = 0,
        Counterclockwise = 1,
    }

    #[repr(u8)]
    #[derive(Debug, Copy, Clone, Serialize, Deserialize)]
    pub enum TorqueMode {
        Disabled = 0,
        Enabled = 1,
        Stiff = 2,
    }

    impl PartialEq for TorqueMode {
        fn eq(&self, other: &Self) -> bool {
            self == other
        }
    }

    #[derive(Debug, Clone, Copy, Serialize, Deserialize)]
    pub struct IMUData {
        pub acc_x: f32,
        pub acc_y: f32,
        pub acc_z: f32,
        pub gyro_x: f32,
        pub gyro_y: f32,
        pub gyro_z: f32,
    }
}
