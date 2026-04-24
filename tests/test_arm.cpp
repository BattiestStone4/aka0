// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka0-ref commits 755a885, 7b7011d
// Description: 交互式三舵机调试工具

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arm.hpp"

int main() {
    Arm arm;

    printf("=== 三个舵机角度调试工具 ===\n");
    printf("输入格式: 角度0 角度1 角度2\n");
    printf("  角度范围: 0~270\n");
    printf("  输入 q 退出\n");
    printf("  输入 grab / release / show / pos 测试预设动作\n\n");

    float a0 = 240, a1 = 220, a2 = 150;
    printf("初始位置: %.0f %.0f %.0f\n", a0, a1, a2);
    arm.set_angle(0, a0);
    arm.set_angle(1, a1);
    arm.set_angle(2, a2);

    char line[128];
    while (true) {
        printf("> ");
        fflush(stdout);
        if (!fgets(line, sizeof(line), stdin)) break;

        // trim newline
        line[strcspn(line, "\n")] = 0;

        if (line[0] == 'q' && (line[1] == 0 || line[1] == '\n')) {
            printf("Bye\n");
            break;
        }

        if (strcmp(line, "grab") == 0) {
            printf("Executing grab...\n");
            arm.grab();
            continue;
        }
        if (strcmp(line, "release") == 0) {
            printf("Releasing gripper...\n");
            arm.release();
            continue;
        }
        if (strcmp(line, "show") == 0) {
            printf("Showing ball...\n");
            arm.show();
            continue;
        }
        if (strcmp(line, "pos") == 0) {
            printf("Moving to grab_pos...\n");
            arm.grab_pos();
            continue;
        }

        float n0, n1, n2;
        if (sscanf(line, "%f %f %f", &n0, &n1, &n2) != 3) {
            printf("格式错误，请输入: 角度0 角度1 角度2\n");
            continue;
        }

        a0 = n0; a1 = n1; a2 = n2;
        printf("servo 0=%.0f 1=%.0f 2=%.0f\n", a0, a1, a2);
        arm.set_angle(0, a0);
        arm.set_angle(1, a1);
        arm.set_angle(2, a2);
    }

    return 0;
}
