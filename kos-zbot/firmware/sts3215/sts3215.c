#include "sts3215.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
// #include "ion.h"
#include "ion_cvitek.h"

#define CVIMMAP_SHMEM_ADDR 0x9fd00000
#define CVIMMAP_SHMEM_SIZE 256
#define ION_DEVICE "/dev/ion"
#define RTOS_CMDQU_DEV_NAME "/dev/cvi-rtos-cmdqu"
#define RTOS_CMDQU_SEND_WAIT _IOW('r', 2, unsigned long)

enum SYS_CMD_ID {
    SYS_CMD_GET_SERVO_VALUES = 0x21,
    SYS_CMD_SERVO_WRITE,
    SYS_CMD_SERVO_READ,
    SYS_CMD_SERVO_READOUT_ENABLE,
    SYS_CMD_SERVO_READOUT_DISABLE,
    SYS_CMD_SERVO_WRITE_MULTIPLE,
    SYS_CMD_SERVO_MOVEMENT_ENABLE,
    SYS_CMD_SERVO_MOVEMENT_DISABLE
};

typedef struct {
    unsigned char ip_id;
    unsigned char cmd_id : 7;
    unsigned char block : 1;
    union {
        struct {
            unsigned char linux_valid;
            unsigned char rtos_valid;
        } valid;
        unsigned short mstime;
    } resv;
    unsigned int param_ptr;
} __attribute__((packed)) __attribute__((aligned(0x8))) cmdqu_t;

static int mailbox_fd = -1;
static int ion_fd = -1;
static int mem_fd = -1;
static void *shared_mem = NULL;

static int perform_mailbox_operation(enum SYS_CMD_ID cmd_id, size_t data_size) {
    struct ion_custom_data custom_data;
    struct cvitek_cache_range range;
    cmdqu_t cmdqu = {0};

    range.paddr = CVIMMAP_SHMEM_ADDR;
    range.size = CVIMMAP_SHMEM_SIZE;

    custom_data.cmd = ION_IOC_CVITEK_FLUSH_PHY_RANGE;
    custom_data.arg = (unsigned long)&range;

    if (ioctl(ion_fd, ION_IOC_CUSTOM, &custom_data) < 0) {
        return -1;
    }

    cmdqu.ip_id = 0;
    cmdqu.cmd_id = cmd_id;
    cmdqu.resv.mstime = 100;
    cmdqu.param_ptr = CVIMMAP_SHMEM_ADDR;

    if (ioctl(mailbox_fd, RTOS_CMDQU_SEND_WAIT, &cmdqu) < 0) {
        return -1;
    }

    custom_data.cmd = ION_IOC_CVITEK_INVALIDATE_PHY_RANGE;
    if (ioctl(ion_fd, ION_IOC_CUSTOM, &custom_data) < 0) {
        return -1;
    }

    return 0;
}

int servo_init() {
    mailbox_fd = open(RTOS_CMDQU_DEV_NAME, O_RDWR);
    if (mailbox_fd <= 0) {
        return -1;
    }

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        close(mailbox_fd);
        return -1;
    }

    shared_mem = mmap(NULL, CVIMMAP_SHMEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, CVIMMAP_SHMEM_ADDR);
    if (shared_mem == MAP_FAILED) {
        close(mem_fd);
        close(mailbox_fd);
        return -1;
    }

    ion_fd = open(ION_DEVICE, O_RDWR);
    if (ion_fd < 0) {
        munmap(shared_mem, CVIMMAP_SHMEM_SIZE);
        close(mem_fd);
        close(mailbox_fd);
        return -1;
    }

    return 0;
}

void servo_deinit() {
    if (shared_mem != NULL) {
        munmap(shared_mem, CVIMMAP_SHMEM_SIZE);
    }
    if (ion_fd >= 0) {
        close(ion_fd);
    }
    if (mem_fd >= 0) {
        close(mem_fd);
    }
    if (mailbox_fd >= 0) {
        close(mailbox_fd);
    }
}

int servo_write(uint8_t id, uint8_t address, uint8_t *data, uint8_t length) {
    typedef struct {
        uint8_t id;
        uint8_t address;
        uint8_t length;
        uint8_t data[MAX_SERVO_COMMAND_DATA];
    } ServoCommand;

    ServoCommand *cmd = (ServoCommand *)shared_mem;
    cmd->id = id;
    cmd->address = address;
    cmd->length = length;
    memcpy(cmd->data, data, length);

    return perform_mailbox_operation(SYS_CMD_SERVO_WRITE, sizeof(ServoCommand));
}

int servo_read(uint8_t id, uint8_t address, uint8_t length, uint8_t *data) {
    typedef struct {
        uint8_t id;
        uint8_t address;
        uint8_t length;
    } ServoCommand;

    ServoCommand *cmd = (ServoCommand *)shared_mem;
    cmd->id = id;
    cmd->address = address;
    cmd->length = length;

    int ret = perform_mailbox_operation(SYS_CMD_SERVO_READ, sizeof(ServoCommand));
    if (ret < 0) {
        return ret;
    }

    memcpy(data, shared_mem + 5, length);
    return 0;
}

int servo_move(uint8_t id, int16_t position, uint16_t time, uint16_t speed) {
    uint8_t data[6];
    data[0] = position & 0xFF;
    data[1] = (position >> 8) & 0xFF;
    data[2] = time & 0xFF;
    data[3] = (time >> 8) & 0xFF;
    data[4] = speed & 0xFF;
    data[5] = (speed >> 8) & 0xFF;
    
    return servo_write(id, 0x2A, data, 6);
}

int enable_servo_readout() {
    return perform_mailbox_operation(SYS_CMD_SERVO_READOUT_ENABLE, 0);
}

int disable_servo_readout() {
    return perform_mailbox_operation(SYS_CMD_SERVO_READOUT_DISABLE, 0);
}

int enable_servo_movement() {
    return perform_mailbox_operation(SYS_CMD_SERVO_MOVEMENT_ENABLE, 0);
}

int disable_servo_movement() {
    return perform_mailbox_operation(SYS_CMD_SERVO_MOVEMENT_DISABLE, 0);
}

int set_servo_mode(uint8_t id, uint8_t mode) {
    uint8_t data = mode;
    return servo_write(id, 0x21, &data, 1);
}

int set_servo_speed(uint8_t id, uint16_t speed, int direction) {
    uint16_t speed_with_direction = speed & 0x7FFF;
    if (direction < 0) {
        speed_with_direction |= 0x8000;
    }
    uint8_t data[2];
    data[0] = speed_with_direction & 0xFF;
    data[1] = (speed_with_direction >> 8) & 0xFF;
    return servo_write(id, 0x2E, data, 2);
}

int servo_read_info(uint8_t id, ServoInfo *info) {
    uint8_t data[30];
    if (servo_read(id, 0x28, 30, data) != 0) {
        return -1;
    }

    info->torque_switch = data[0];
    info->acceleration = data[1];
    info->target_location = (int16_t)((data[3] << 8) | data[2]);
    info->running_time = (uint16_t)((data[5] << 8) | data[4]);
    info->running_speed = (uint16_t)((data[7] << 8) | data[6]);
    info->torque_limit = (uint16_t)((data[9] << 8) | data[8]);
    info->lock_mark = data[15];
    info->current_location = (int16_t)((data[17] << 8) | data[16]);
    info->current_speed = (int16_t)((data[19] << 8) | data[18]);
    info->current_load = (int16_t)((data[21] << 8) | data[20]);
    info->current_voltage = data[22];
    info->current_temperature = data[23];
    info->async_write_flag = data[24];
    info->servo_status = data[25];
    info->mobile_sign = data[26];
    info->current_current = (uint16_t)((data[29] << 8) | data[28]);

    return 0;
}

int read_servo_positions(ServoData *servo_data) {
    if (perform_mailbox_operation(SYS_CMD_GET_SERVO_VALUES, 0) < 0) {
        return -1;
    }

    // Invalidate the cache
    struct ion_custom_data custom_data;
    struct cvitek_cache_range range;

    range.paddr = CVIMMAP_SHMEM_ADDR;
    range.size = sizeof(ServoData);

    custom_data.cmd = ION_IOC_CVITEK_INVALIDATE_PHY_RANGE;
    custom_data.arg = (unsigned long)&range;

    if (ioctl(ion_fd, ION_IOC_CUSTOM, &custom_data) < 0) {
        return -1;
    }

    // Read ServoData from shared memory
    memcpy(servo_data, shared_mem, sizeof(ServoData));

    return 0;
}

int servo_write_multiple(ServoMultipleWriteCommand *cmd) {
    // Copy the command to shared memory
    memcpy(shared_mem, cmd, sizeof(ServoMultipleWriteCommand));

    // Perform the mailbox operation
    int ret = perform_mailbox_operation(SYS_CMD_SERVO_WRITE_MULTIPLE, sizeof(ServoMultipleWriteCommand));
    if (ret < 0) {
        printf("c: Error performing mailbox operation, ret: %d\n", ret);
        return ret;
    }

    return 0;
}