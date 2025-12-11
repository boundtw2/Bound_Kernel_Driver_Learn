#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <time.h>

// 这个结构体必须和内核驱动中的定义完全一致
struct mma8653_data_t {
    short x;
    short y;
    short z;
};

// MMA8653 在 ±2g 量程下的灵敏度
#define SENSITIVITY_2G (1.0f / 256.0f)

// 算法阈值参数
#define GRAVITY_G 1.0f
// 跌倒检测阈值
#define FALL_FREE_FALL_THRESHOLD 0.6f  // 失重阈值 (小于此值可能在下落)
#define FALL_IMPACT_THRESHOLD    2.5f  // 撞击阈值 (大于此值可能是撞击地面)
// 计步阈值
#define STEP_THRESHOLD           1.2f  // 走路时合加速度波峰通常大于此值
#define STEP_MIN_INTERVAL_MS     300   // 两步之间最小时间间隔(毫秒)，防止抖动误判

// 状态管理
typedef struct {
    int step_count;
    long long last_step_time;
    int possible_fall_stage; // 0: 正常, 1: 检测到失重, 2: 检测到撞击
    long long fall_stage_timestamp;
} AppState;

// 获取当前时间戳(毫秒)
long long current_timestamp() {
    struct timespec te;
    clock_gettime(CLOCK_REALTIME, &te);
    long long milliseconds = te.tv_sec * 1000LL + te.tv_nsec / 1000000;
    return milliseconds;
}

int main() {
    int fd;
    struct mma8653_data_t raw_data;
    ssize_t ret;
    AppState app_state = {0, 0, 0, 0};

    const char *device_node = "/dev/my_mma8653";

    // 打开设备文件
    fd = open(device_node, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open device file");
        return -1;
    }

    printf("=== Smart Care Monitor Started ===\n");
    printf("Device: %s\n", device_node);
    printf("Functions: Step Counting & Fall Detection\n\n");

    // 循环读取和处理
    while (1) {
        // 1. 读取数据
        ret = read(fd, &raw_data, sizeof(struct mma8653_data_t));
        if (ret != sizeof(struct mma8653_data_t)) {
            usleep(10000); // 读取错误稍作等待
            continue;
        }

        // 2. 转换为物理量 (g)
        float ax = raw_data.x * SENSITIVITY_2G;
        float ay = raw_data.y * SENSITIVITY_2G;
        float az = raw_data.z * SENSITIVITY_2G;

        // 3. 计算合加速度 (SVM - Signal Vector Magnitude)
        // SVM 反映了受到的总合力，不受设备方向影响（只要重力存在，静止就是1g）
        float svm = sqrt(ax*ax + ay*ay + az*az);
        long long now = current_timestamp();

        // --- 算法部分 ---

        // A. 简易计步算法
        // 逻辑：检测波峰。当合力超过阈值，且距离上一步时间足够长
        if (svm > STEP_THRESHOLD) {
            if (now - app_state.last_step_time > STEP_MIN_INTERVAL_MS) {
                app_state.step_count++;
                app_state.last_step_time = now;
                printf("\n[EVENT] Step Detected! Total Steps: %d\n", app_state.step_count);
            }
        }

        // B. 跌倒检测算法 (失重 -> 撞击 序列检测)
        // 阶段 1: 监测失重 (Fall Start)
        if (app_state.possible_fall_stage == 0) {
            if (svm < FALL_FREE_FALL_THRESHOLD) {
                app_state.possible_fall_stage = 1; // 进入失重疑似阶段
                app_state.fall_stage_timestamp = now;
                // printf("DEBUG: Potential fall detected (Free fall)\n");
            }
        }
        // 阶段 2: 监测撞击 (Impact)
        else if (app_state.possible_fall_stage == 1) {
            // 如果失重后 500ms 内没有发生撞击，则认为是误判（可能是扔了一下手机），重置
            if (now - app_state.fall_stage_timestamp > 500) {
                app_state.possible_fall_stage = 0;
            } 
            else if (svm > FALL_IMPACT_THRESHOLD) {
                // 检测到剧烈撞击，确认为跌倒
                printf("\n\033[1;31m[ALARM] FALL DETECTED! (Free fall -> Impact)\033[0m\n");
                printf("Impact Force: %.2fg\n", svm);
                // 重置状态
                app_state.possible_fall_stage = 0;
                // 这里可以触发蜂鸣器或发送网络请求
                sleep(2); // 暂停一下避免连续刷屏
            }
        }

        // 4. 实时状态显示
        // \r 原地刷新，只显示 SVM 和 步数
        printf("\rSVM: %.2fg | Steps: %d | Status: %s", 
               svm, 
               app_state.step_count,
               (svm < 0.8 || svm > 1.2) ? "Moving" : "Static");
        
        fflush(stdout);

        // 采样频率控制：50Hz (20ms)
        // usleep(20000); 
        // 注意：您的驱动里写了 msleep(100) 并没有在 read 里阻塞等待数据就绪
        // 为了演示效果，我们在应用层稍微快一点读取
        usleep(20000); 
    }

    close(fd);
    return 0;
}
