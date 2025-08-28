#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

// 这个结构体必须和内核驱动中的定义完全一致
struct mma8653_data_t {
    short x;
    short y;
    short z;
};

// MMA8653 在 ±2g 量程下的灵敏度 (LSB/g)
// 10位数据，所以是 2^10 / (2 * 2g) = 1024 / 4 = 256 LSB/g
// 所以，1 LSB = 1/256 g
#define SENSITIVITY_2G (1.0f / 256.0f)

int main() {
    int fd;
    struct mma8653_data_t raw_data;
    ssize_t ret;

    const char *device_node = "/dev/simple_mma8653";

    // 打开设备文件
    fd = open(device_node, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open device file");
        return -1;
    }

    printf("Device %s opened successfully. Reading data...\n", device_node);

    // 循环读取和显示数据
    while (1) {
        // 从驱动读取原始数据
        ret = read(fd, &raw_data, sizeof(struct mma8653_data_t));
        if (ret < 0) {
            perror("Failed to read from device");
            break;
        }

        if (ret != sizeof(struct mma8653_data_t)) {
            fprintf(stderr, "Error: read size mismatch. Expected %zu, got %zd\n", 
                    sizeof(struct mma8653_data_t), ret);
            break;
        }

        // 在用户空间进行浮点转换
        float accel_x = raw_data.x * SENSITIVITY_2G;
        float accel_y = raw_data.y * SENSITIVITY_2G;
        float accel_z = raw_data.z * SENSITIVITY_2G;

        // 打印转换后的值 (单位: g)
        // \r 用于将光标移到行首，实现原地更新
        printf("\rAcceleration (g): X=%.4f, Y=%.4f, Z=%.4f", accel_x, accel_y, accel_z);
        fflush(stdout); // 确保立即输出

        // 每 200 毫秒读取一次
        usleep(200000); 
    }

    // 关闭设备
    close(fd);
    printf("\nDevice closed.\n");

    return 0;
}

