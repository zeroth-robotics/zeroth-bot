#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define EFUSE_DEVICE "/dev/efuse"
#define EFUSE_START_OFFSET 8
#define EFUSE_READ_SIZE 32
#define MAC_SIZE 6

#define EFUSE_IOC_MAGIC 'E'
#define EFUSE_IOC_READ _IOR(EFUSE_IOC_MAGIC, 1, struct efuse_data)

struct efuse_data {
    uint32_t addr;
    uint32_t value;
};

// Simple hash function
uint32_t hash(const unsigned char *data, size_t len) {
    uint32_t hash = 5381;
    for (size_t i = 0; i < len; i++) {
        hash = ((hash << 5) + hash) + data[i];
    }
    return hash;
}

int read_efuse(int fd, uint32_t addr, uint32_t *value) {
    struct efuse_data data = {
        .addr = addr
    };
    int ret = ioctl(fd, EFUSE_IOC_READ, &data);
    if (ret < 0) {
        return ret;
    }
    *value = data.value;
    return 0;
}

void generate_mac_addresses(const unsigned char *efuse_data, unsigned char *mac1, unsigned char *mac2) {
    uint32_t hash_value = hash(efuse_data, EFUSE_READ_SIZE);
    
    // First MAC address
    mac1[0] = 0x02;  // Locally administered
    mac1[1] = (hash_value >> 24) & 0xFF;
    mac1[2] = (hash_value >> 16) & 0xFF;
    mac1[3] = (hash_value >> 8) & 0xFF;
    mac1[4] = hash_value & 0xFF;
    mac1[5] = (hash_value >> 4) & 0xFF;

    // Second MAC address
    mac2[0] = 0x02;  // Locally administered
    mac2[1] = (hash_value >> 20) & 0xFF;
    mac2[2] = (hash_value >> 12) & 0xFF;
    mac2[3] = (hash_value >> 4) & 0xFF;
    mac2[4] = ((hash_value & 0xF) << 4) | ((hash_value >> 28) & 0xF);
    mac2[5] = (hash_value >> 16) & 0xFF;
}

int main() {
    printf("Reading efuse data...\n");
    int fd = open(EFUSE_DEVICE, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open efuse device");
        return 1;
    }

    unsigned char efuse_data[EFUSE_READ_SIZE];
    uint32_t value;

    // Read 32 bytes from efuse, starting at offset 8
    for (int i = 0; i < EFUSE_READ_SIZE / 4; i++) {
        if (read_efuse(fd, EFUSE_START_OFFSET + i*4, &value) < 0) {
            perror("Failed to read from efuse");
            close(fd);
            return 1;
        }
        efuse_data[i*4] = (value >> 24) & 0xFF;
        efuse_data[i*4 + 1] = (value >> 16) & 0xFF;
        efuse_data[i*4 + 2] = (value >> 8) & 0xFF;
        efuse_data[i*4 + 3] = value & 0xFF;
    }
    printf("Read efuse data: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
           efuse_data[0], efuse_data[1], efuse_data[2], efuse_data[3], efuse_data[4], efuse_data[5],
           efuse_data[6], efuse_data[7], efuse_data[8], efuse_data[9], efuse_data[10], efuse_data[11],
           efuse_data[12], efuse_data[13], efuse_data[14], efuse_data[15]);

    close(fd);

    printf("Generating MAC addresses...\n");
    unsigned char mac1[MAC_SIZE], mac2[MAC_SIZE];
    generate_mac_addresses(efuse_data, mac1, mac2);

    printf("Generated MAC addresses based on efuse data:\n");
    printf("MAC1: %02X:%02X:%02X:%02X:%02X:%02X\n", mac1[0], mac1[1], mac1[2], mac1[3], mac1[4], mac1[5]);
    printf("MAC2: %02X:%02X:%02X:%02X:%02X:%02X\n", mac2[0], mac2[1], mac2[2], mac2[3], mac2[4], mac2[5]);

    // Write MAC addresses to files
    FILE *fp1 = fopen("/tmp/mac1", "w");
    FILE *fp2 = fopen("/tmp/mac2", "w");
    if (fp1 == NULL || fp2 == NULL) {
        perror("Failed to open output files");
        return 1;
    }

    fprintf(fp1, "%02X:%02X:%02X:%02X:%02X:%02X\n", mac1[0], mac1[1], mac1[2], mac1[3], mac1[4], mac1[5]);
    fprintf(fp2, "%02X:%02X:%02X:%02X:%02X:%02X\n", mac2[0], mac2[1], mac2[2], mac2[3], mac2[4], mac2[5]);

    fclose(fp1);
    fclose(fp2);

    printf("MAC addresses written to /tmp/mac1 and /tmp/mac2\n");

    return 0;
}