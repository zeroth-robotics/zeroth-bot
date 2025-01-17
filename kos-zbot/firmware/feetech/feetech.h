#ifndef FEETECH_H
#define FEETECH_H

#include <stdint.h>

#define MAX_SERVO_COMMAND_DATA 40
#define MAX_SHMEM_DATA 2048
#define MAX_SERVOS 32

typedef struct {
    uint8_t id;
    uint32_t last_read_ms;
    uint8_t torque_switch;         // 0x28 (1 byte)
    uint8_t acceleration;          // 0x29 (1 byte)
    int16_t target_location;       // 0x2A (2 bytes)
    uint16_t running_time;         // 0x2C (2 bytes)
    uint16_t running_speed;        // 0x2E (2 bytes)
    uint16_t torque_limit;         // 0x30 (2 bytes)
    uint8_t reserved1[6];          // 0x32-0x37 (6 bytes, reserved)
    uint8_t lock_mark;             // 0x37 (1 byte)
    int16_t current_location;      // 0x38 (2 bytes)
    int16_t current_speed;         // 0x3A (2 bytes)
    int16_t current_load;          // 0x3C (2 bytes)
    uint8_t current_voltage;       // 0x3E (1 byte)
    uint8_t current_temperature;   // 0x3F (1 byte)
    uint8_t async_write_flag;      // 0x40 (1 byte)
    uint8_t servo_status;          // 0x41 (1 byte)
    uint8_t mobile_sign;           // 0x42 (1 byte)
    uint8_t reserved2[2];          // 0x43-0x44 (2 bytes, reserved)
    uint16_t current_current;      // 0x45 (2 bytes)
} ServoInfo;

typedef struct {
    uint8_t id;
    uint8_t address;
    uint8_t length;
    uint8_t data[MAX_SERVO_COMMAND_DATA];
} ServoCommand;

typedef struct {
    uint32_t data_length;
    uint8_t data[MAX_SHMEM_DATA];
} BroadcastCommand;

typedef struct {
    uint32_t retry_count;
    uint32_t read_count;
    uint32_t loop_count;
    uint32_t fault_count;
    uint32_t last_read_ms;
    ServoInfo servos[MAX_SERVOS];
} ServoInfoBuffer;

typedef struct {
    uint32_t len;
    uint8_t servo_id[MAX_SERVOS];
} ActiveServoList;

enum SYS_CMD_ID {
    SYS_CMD_SERVO_READ = 0x21,
	SYS_CMD_SERVO_WRITE,
	SYS_CMD_SERVO_INFO,
	SYS_CMD_SERVO_BROADCAST,
	SYS_CMD_SET_ACTIVE_SERVO_LIST,
};

// Write data to a servo
int servo_write(uint8_t id, uint8_t address, uint8_t *data, uint8_t length);

// Read data from a servo
int servo_read(uint8_t id, uint8_t address, uint8_t *data, uint8_t length);

int servo_set_active_servos(ActiveServoList active_servos);

int servo_get_info(ServoInfoBuffer *info_buffer);

#endif