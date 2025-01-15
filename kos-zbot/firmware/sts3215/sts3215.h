#ifndef SERVO_LIB_H
#define SERVO_LIB_H

#include <stdint.h>

#define MAX_SERVOS 16
#define MAX_SERVO_COMMAND_DATA 256

typedef struct {
    uint8_t torque_switch;
    uint8_t acceleration;
    int16_t target_location;
    uint16_t running_time;
    uint16_t running_speed;
    uint16_t torque_limit;
    uint8_t reserved1[6];
    uint8_t lock_mark;
    int16_t current_location;
    int16_t current_speed;
    int16_t current_load;
    uint8_t current_voltage;
    uint8_t current_temperature;
    uint8_t async_write_flag;
    uint8_t servo_status;
    uint8_t mobile_sign;
    uint8_t reserved2[2];
    uint16_t current_current;
} ServoInfo;

typedef struct {
    ServoInfo servo[MAX_SERVOS];
    uint32_t task_run_count;
} ServoData;

typedef struct {
    uint8_t only_write_positions;
    uint8_t ids[MAX_SERVOS];
    int16_t positions[MAX_SERVOS];
    uint16_t times[MAX_SERVOS];
    uint16_t speeds[MAX_SERVOS];
} ServoMultipleWriteCommand;

// Initialize the servo library
int servo_init();

// Deinitialize the servo library
void servo_deinit();

// Write data to a servo
int servo_write(uint8_t id, uint8_t address, uint8_t *data, uint8_t length);

// Read data from a servo
int servo_read(uint8_t id, uint8_t address, uint8_t length, uint8_t *data);

// Move a servo to a specific position
int servo_move(uint8_t id, int16_t position, uint16_t time, uint16_t speed);

// Enable servo readout
int enable_servo_readout();

// Disable servo readout
int disable_servo_readout();

// Enable servo movement
int enable_servo_movement();

// Disable servo movement
int disable_servo_movement();

// Set servo mode
int set_servo_mode(uint8_t id, uint8_t mode);

// Set servo speed
int set_servo_speed(uint8_t id, uint16_t speed, int direction);

// Read servo info
int servo_read_info(uint8_t id, ServoInfo *info);

int read_servo_positions(ServoData *servo_data);

// Add this new function declaration
int servo_write_multiple(ServoMultipleWriteCommand *cmd);

#endif // SERVO_LIB_H