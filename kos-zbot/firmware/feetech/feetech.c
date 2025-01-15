#include "feetech.h"
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

int servo_set_active_servos(ActiveServoList active_servos) {
    ActiveServoList *cmd = (ActiveServoList *)shared_mem;
    cmd->len = active_servos.len;
    memcpy(cmd->servo_id, active_servos.servo_id, active_servos.len);
    return perform_mailbox_operation(SYS_CMD_SET_ACTIVE_SERVO_LIST, sizeof(ActiveServoList));
}

int servo_get_info(ServoInfoBuffer *info_buffer) {
    perform_mailbox_operation(SYS_CMD_SERVO_INFO, sizeof(ServoInfoBuffer));
    memcpy(info_buffer, shared_mem, sizeof(ServoInfoBuffer));
    return 0;
}

int servo_broadcast_command(BroadcastCommand command) {
    BroadcastCommand *cmd = (BroadcastCommand *)shared_mem;
    memcpy(cmd, &command, sizeof(BroadcastCommand));
    return perform_mailbox_operation(SYS_CMD_SERVO_BROADCAST, sizeof(BroadcastCommand));
}
