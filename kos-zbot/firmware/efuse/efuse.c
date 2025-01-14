#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#define EFUSE_DEVICE "/dev/efuse"
#define DUMP_SIZE 256

int main(int argc, char *argv[]) {
    int fd;
    ssize_t bytes_read;
    char buffer[DUMP_SIZE];
    FILE *output_file;

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <output_file>\n", argv[0]);
        exit(1);
    }

    // Open the efuse device
    fd = open(EFUSE_DEVICE, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open efuse device");
        exit(1);
    }

    // Read 2KB from the device
    bytes_read = read(fd, buffer, DUMP_SIZE);
    if (bytes_read < 0) {
        perror("Failed to read from efuse device");
        close(fd);
        exit(1);
    }

    // Close the device
    close(fd);

    // Open the output file
    output_file = fopen(argv[1], "wb");
    if (output_file == NULL) {
        perror("Failed to open output file");
        exit(1);
    }

    // Write the data to the output file
    if (fwrite(buffer, 1, bytes_read, output_file) != (size_t)bytes_read) {
        perror("Failed to write to output file");
        fclose(output_file);
        exit(1);
    }

    // Close the output file
    fclose(output_file);

    printf("Successfully read %zd bytes from efuse and saved to %s\n", bytes_read, argv[1]);

    return 0;
}