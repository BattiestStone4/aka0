/*
 * Camera test utility (works on both Starry and Linux UART mode).
 *
 * Usage:
 *   ./camera_test                       # capture one frame -> /tmp/camera_test.jpg
 *   ./camera_test -n 10                 # capture 10 frames
 *   ./camera_test -o /tmp/test.jpg      # specify output path
 *   ./camera_test --benchmark           # continuous capture with FPS stats
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include "vi_capture.hpp"

static volatile int g_running = 1;

static void signal_handler(int sig) {
    g_running = 0;
}

static void usage(const char *prog) {
#if USE_UART_CAMERA
    printf("Camera test (UART ESP32-CAM)\n\n");
    printf("Options:\n");
    printf("  --dev DEV       UART device (default: %s, or UART_DEV env)\n", UART_DEFAULT_DEV);
    printf("  --baud RATE     Baudrate (default: %d, or UART_BAUD env)\n", UART_DEFAULT_BAUD);
#else
    printf("Camera test (Starry VI)\n\n");
#endif
    printf("Usage: %s [options]\n\n", prog);
    printf("  -n COUNT        Number of frames to capture (default: 1, 0=forever)\n");
    printf("  -o PATH         Output JPEG path (default: /tmp/camera_test.jpg)\n");
    printf("  --benchmark     Continuous capture with FPS stats\n");
}

int main(int argc, char **argv) {
#if USE_UART_CAMERA
    const char *dev = getenv("UART_DEV");
    if (!dev) dev = UART_DEFAULT_DEV;
    int baud = UART_DEFAULT_BAUD;
    const char *baud_env = getenv("UART_BAUD");
    if (baud_env) baud = atoi(baud_env);
#endif

    int count = 1;
    const char *out_path = "/tmp/camera_test.jpg";
    bool benchmark = false;

    for (int i = 1; i < argc; i++) {
#if USE_UART_CAMERA
        if (strcmp(argv[i], "--dev") == 0 && i + 1 < argc) {
            dev = argv[++i];
        } else if (strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            baud = atoi(argv[++i]);
        } else
#endif
        if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else if (strcmp(argv[i], "--benchmark") == 0) {
            benchmark = true;
            count = 0;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

#if USE_UART_CAMERA
    setenv("UART_DEV", dev, 1);
    char baud_str[16];
    snprintf(baud_str, sizeof(baud_str), "%d", baud);
    setenv("UART_BAUD", baud_str, 1);
    printf("=== Camera Test (UART %s @ %d) ===\n", dev, baud);
#else
    printf("=== Camera Test (Starry VI) ===\n");
#endif

    VICapture cam;
    if (cam.init() != CVI_SUCCESS) {
        fprintf(stderr, "Failed to init camera\n");
        return 1;
    }
    printf("Camera initialized OK\n");

#if USE_VPSS_RESIZE
    if (cam.initVpssResize(2560, 1440, 640, 640) != CVI_SUCCESS) {
        fprintf(stderr, "Failed to init VPSS\n");
        cam.deinit();
        return 1;
    }
    printf("VPSS initialized OK (2560x1440 -> 640x640)\n");
    usleep(500 * 1000);
#endif

    // Ensure output directory exists
    {
        std::string path(out_path);
        size_t pos = path.rfind('/');
        if (pos != std::string::npos)
            mkdir(path.substr(0, pos).c_str(), 0755);
    }

    if (benchmark) {
        printf("Benchmark mode: capturing until Ctrl-C...\n\n");
        int frame_idx = 0;
        int fail_count = 0;
        struct timeval t_start, t_last, t_now;
        gettimeofday(&t_start, NULL);
        t_last = t_start;

        while (g_running) {
            cv::Mat frame;
            struct timeval t0, t1;
            gettimeofday(&t0, NULL);
            int ret = cam.getFrameAsBGR(0, frame);
            gettimeofday(&t1, NULL);
            frame_idx++;

            long capture_ms = (t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_usec - t0.tv_usec) / 1000;

            if (ret == CVI_SUCCESS && !frame.empty()) {
                gettimeofday(&t_now, NULL);
                float fps_inst = 1000000.0f / ((t_now.tv_sec - t_last.tv_sec) * 1000000 +
                                                 (t_now.tv_usec - t_last.tv_usec));
                float fps_avg = frame_idx * 1000000.0f / ((t_now.tv_sec - t_start.tv_sec) * 1000000 +
                                                            (t_now.tv_usec - t_start.tv_usec));
                printf("[%d] %dx%d  %ldms  FPS: %.1f  avg: %.1f\n",
                       frame_idx, frame.cols, frame.rows, capture_ms, fps_inst, fps_avg);
                t_last = t_now;
            } else {
                fail_count++;
                printf("[%d] FAILED (%ldms)\n", frame_idx, capture_ms);
            }
        }

        printf("\nBenchmark: %d frames (%d failed)\n", frame_idx - fail_count, fail_count);
    } else {
        // Single/multi frame capture
        int ok_count = 0, fail_count = 0;

        for (int i = 0; i < count && g_running; i++) {
            cv::Mat frame;
            struct timeval t0, t1;
            gettimeofday(&t0, NULL);
            int ret = cam.getFrameAsBGR(0, frame);
            gettimeofday(&t1, NULL);
            long capture_ms = (t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_usec - t0.tv_usec) / 1000;

            if (ret != CVI_SUCCESS || frame.empty()) {
                fprintf(stderr, "[%d/%d] FAILED (%ldms)\n", i + 1, count, capture_ms);
                fail_count++;
                continue;
            }

            ok_count++;
            printf("[%d/%d] %dx%d  %ldms\n", i + 1, count, frame.cols, frame.rows, capture_ms);

            char path[256];
            if (count == 1)
                snprintf(path, sizeof(path), "%s", out_path);
            else
                snprintf(path, sizeof(path), "/tmp/camera_test_%03d.jpg", i + 1);

            if (cv::imwrite(path, frame))
                printf("  saved: %s\n", path);
            else
                fprintf(stderr, "  save FAILED: %s\n", path);
        }

        printf("\nResult: %d ok, %d failed\n", ok_count, fail_count);
    }

#if USE_VPSS_RESIZE
    cam.deinitVpssResize();
#endif
    cam.deinit();
    return 0;
}
