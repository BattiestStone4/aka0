#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "arm.hpp"

int main() {
    fprintf(stderr, "[arm_test] 1. main() entered\n");
    fflush(stderr);

    // Workaround: open console directly (StarryOS stdin doesn't reach child processes)
    int input_fd = open("/dev/console", O_RDWR);
    if (input_fd < 0) {
        fprintf(stderr, "[arm_test] cannot open /dev/console, using stdin\n");
        fflush(stderr);
        input_fd = 0;
    } else {
        fprintf(stderr, "[arm_test] using /dev/console for input (fd=%d)\n", input_fd);
        fflush(stderr);
    }

    fprintf(stderr, "[arm_test] 2. creating Arm object...\n");
    fflush(stderr);
    Arm arm;
    fprintf(stderr, "[arm_test] 3. Arm created\n");
    fflush(stderr);

    printf("=== 三个舵机角度调试工具 ===\n");
    printf("输入格式: 角度0 角度1 角度2\n");
    printf("  角度范围: 0~270\n");
    printf("  输入 q 退出\n");
    printf("  输入 grab / release / show / pos 测试预设动作\n\n");
    fflush(stdout);

    float a0 = 240, a1 = 220, a2 = 150;
    printf("初始位置: %.0f %.0f %.0f\n", a0, a1, a2);
    fflush(stdout);

    arm.set_angle(0, a0);
    arm.set_angle(1, a1);
    arm.set_angle(2, a2);
    fprintf(stderr, "[arm_test] 4. entering loop\n");
    fflush(stderr);

    char line[128];
    while (true) {
        printf("> ");
        fflush(stdout);

        int i = 0;
        bool got_input = false;
        while (i < (int)sizeof(line) - 1) {
            char c;
            ssize_t n = read(input_fd, &c, 1);
            if (n <= 0) {
                fprintf(stderr, "[arm_test] read returned %zd, exiting\n", n);
                goto done;
            }
            if (c == '\n' || c == '\r') {
                got_input = true;
                break;
            }
            line[i++] = c;
        }
        line[i] = 0;

        if (!got_input && i == 0) continue;

        if (line[0] == 'q' && line[1] == 0) {
            printf("Bye\n");
            break;
        }

        if (strcmp(line, "grab") == 0) {
            printf("Executing grab...\n"); fflush(stdout);
            arm.grab();
            continue;
        }
        if (strcmp(line, "release") == 0) {
            printf("Releasing gripper...\n"); fflush(stdout);
            arm.release();
            continue;
        }
        if (strcmp(line, "show") == 0) {
            printf("Showing ball...\n"); fflush(stdout);
            arm.show();
            continue;
        }
        if (strcmp(line, "pos") == 0) {
            printf("Moving to grab_pos...\n"); fflush(stdout);
            arm.grab_pos();
            continue;
        }

        float n0, n1, n2;
        if (sscanf(line, "%f %f %f", &n0, &n1, &n2) != 3) {
            printf("格式错误，请输入: 角度0 角度1 角度2\n"); fflush(stdout);
            continue;
        }

        a0 = n0; a1 = n1; a2 = n2;
        printf("servo 0=%.0f 1=%.0f 2=%.0f\n", a0, a1, a2);
        fflush(stdout);
        arm.set_angle(0, a0);
        arm.set_angle(1, a1);
        arm.set_angle(2, a2);
    }

done:
    if (input_fd > 0) close(input_fd);
    return 0;
}
