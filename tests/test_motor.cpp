// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka0-ref commit 64f8dab
// Description: 电机测试工具，自动测试所有电机动作

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include "motor.hpp"
#include "logger.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage:\n");
        printf("  %s forward [speed]    - 前进 (默认50)\n", argv[0]);
        printf("  %s backward [speed]   - 后退\n", argv[0]);
        printf("  %s left [speed]       - 左转\n", argv[0]);
        printf("  %s right [speed]      - 右转\n", argv[0]);
        printf("  %s drive <L> <R>      - 差速驱动 (正前负后, 范围-100~100)\n", argv[0]);
        printf("  %s stop               - 停车\n", argv[0]);
        printf("  %s test               - 自动测试全部动作\n", argv[0]);
        return 0;
    }

    Motor motor;

    if (strcmp(argv[1], "test") == 0) {
        int speed = 50;
        printf("=== 电机自动测试 ===\n");

        printf("[1/5] 前进 (speed=%d, 2秒)...\n", speed);
        motor.forward(speed);
        sleep(2);

        printf("[2/5] 后退 (speed=%d, 2秒)...\n", speed);
        motor.backward(speed);
        sleep(2);

        printf("[3/5] 左转 (speed=%d, 2秒)...\n", speed);
        motor.left(speed);
        sleep(2);

        printf("[4/5] 右转 (speed=%d, 2秒)...\n", speed);
        motor.right(speed);
        sleep(2);

        printf("[5/5] 差速驱动 L=60 R=30 (2秒)...\n");
        motor.drive(60, 30);
        sleep(2);

        motor.standby();
        printf("=== 测试完成 ===\n");
        return 0;
    }

    int speed = (argc >= 3) ? atoi(argv[2]) : 50;

    if (strcmp(argv[1], "forward") == 0) {
        printf("前进 speed=%d\n", speed);
        motor.forward(speed);
    } else if (strcmp(argv[1], "backward") == 0) {
        printf("后退 speed=%d\n", speed);
        motor.backward(speed);
    } else if (strcmp(argv[1], "left") == 0) {
        printf("左转 speed=%d\n", speed);
        motor.left(speed);
    } else if (strcmp(argv[1], "right") == 0) {
        printf("右转 speed=%d\n", speed);
        motor.right(speed);
    } else if (strcmp(argv[1], "drive") == 0 && argc >= 4) {
        int l = atoi(argv[2]);
        int r = atoi(argv[3]);
        printf("差速驱动 L=%d R=%d\n", l, r);
        motor.drive(l, r);
    } else if (strcmp(argv[1], "stop") == 0) {
        printf("停车\n");
        motor.standby();
    }

    printf("按 Ctrl-C 停止\n");
    while (true) {
        sleep(1);
    }
    return 0;
}
