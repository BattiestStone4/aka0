// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka0-ref commits 64f8dab, 7b7011d, 9c69f3f, d4fbdad
// Description: 完整的追球抓取状态机 - 两状态(chase/grab) + 脉冲式转向 + 动态脉冲 + 太近后退 + Ctrl-C安全退出
// 
// 主要特性：
// - 两状态机: STATUS_CHASE_TENNIS, STATUS_GRAB_TENNIS
// - 使用 area_ratio (bbox面积/图像面积) 判断距离
// - 脉冲式转向防振荡，动态脉冲时间(与area线性相关)
// - 差速原地转向替代单轮转弯
// - 太近(area>0.55)自动后退，避免挤到球
// - 抓取前左转补偿，修正夹爪偏右
// - 完整抓取序列: 伸下→夹紧→抬起→停2秒→松开→复位
// - Ctrl-C 信号处理，安全清理资源

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include "cviruntime.h"
#include "motor.hpp"
#include "arm.hpp"

// VI related headers - must include base types first
#include "vi_capture.hpp"
#include "logger.hpp"

// 控制宏定义
#define ENABLE_DRAW_BBOX 0    // 是否画框并保存图片
#define ENABLE_SAVE_IMAGE 0   // 是否保存检测结果图片

typedef struct {
    float x, y, w, h;
} box;

typedef struct {
    box bbox;
    int cls;
    float score;
    int batch_idx;
} detection;

static const char* tennis_names[] = {"tennis"}; // 单类别网球检测

// 全局指针，用于信号处理中清理资源
static VICapture* g_vi_capture = nullptr;
static Motor* g_motor = nullptr;
static CVI_MODEL_HANDLE g_model = nullptr;

static void cleanup_and_exit() {
    if (g_motor) g_motor->standby();
#if USE_VPSS_RESIZE
    if (g_vi_capture) g_vi_capture->deinitVpssResize();
#endif
    if (g_vi_capture) g_vi_capture->deinit();
    if (g_model) CVI_NN_CleanupModel(g_model);
}

static void signal_handler(int sig) {
    cleanup_and_exit();
    exit(0);
}


static void usage(char** argv) {
    LOGI("Usage:");
    LOGI("   %s cvimodel [vi_channel]", argv[0]);
    LOGI("   Example: %s model.cvimodel 0", argv[0]);
    LOGI("   This will capture video from VI channel (default: 0)");
}

template <typename T> int argmax(const T* data, size_t len, size_t stride = 1) {
    int maxIndex = 0;
    for (size_t i = stride; i < len; i += stride) {
        if (data[maxIndex] < data[i]) {
            maxIndex = i;
        }
    }
    return maxIndex;
}

float calIou(box a, box b) {
    float area1 = a.w * a.h;
    float area2 = b.w * b.h;
    float wi = std::min((a.x + a.w / 2), (b.x + b.w / 2)) - std::max((a.x - a.w / 2), (b.x - b.w / 2));
    float hi = std::min((a.y + a.h / 2), (b.y + b.h / 2)) - std::max((a.y - a.h / 2), (b.y - b.h / 2));
    float area_i = std::max(wi, 0.0f) * std::max(hi, 0.0f);
    return area_i / (area1 + area2 - area_i);
}

static void NMS(std::vector<detection>& dets, int* total, float thresh) {
    if (*total) {
        std::sort(dets.begin(), dets.end(), [](detection& a, detection& b) { return b.score < a.score; });
        int new_count = *total;
        for (int i = 0; i < *total; ++i) {
            detection& a = dets[i];
            if (a.score == 0)
                continue;
            for (int j = i + 1; j < *total; ++j) {
                detection& b = dets[j];
                if (dets[i].batch_idx == dets[j].batch_idx && b.score != 0 && dets[i].cls == dets[j].cls &&
                    calIou(a.bbox, b.bbox) > thresh) {
                    b.score = 0;
                    new_count--;
                }
            }
        }
        std::vector<detection>::iterator it = dets.begin();
        while (it != dets.end()) {
            if (it->score == 0) {
                dets.erase(it);
            } else {
                it++;
            }
        }
        *total = new_count;
    }
}

void correctYoloBoxes(std::vector<detection>& dets, int det_num, int image_h, int image_w, int input_height,
                      int input_width) {
    // 计算缩放比例和padding，与Python代码保持一致
    float scale = std::min((float)input_width / image_w, (float)input_height / image_h);
    int new_h = (int)(image_h * scale);
    int new_w = (int)(image_w * scale);
    int pad_top = (input_height - new_h) / 2;
    int pad_left = (input_width - new_w) / 2;

    LOGD("=== Coordinate correction ===");
    LOGD("Original image: %dx%d, Input size: %dx%d", image_w, image_h, input_width, input_height);
    LOGD("Scale: %.3f, New size: %dx%d, Padding: left=%d, top=%d", scale, new_w, new_h, pad_left, pad_top);

    for (int i = 0; i < det_num; ++i) {
        // YOLOv8输出的是中心点坐标(cx,cy)和宽高(w,h)
        float cx = dets[i].bbox.x;
        float cy = dets[i].bbox.y;
        float w = dets[i].bbox.w;
        float h = dets[i].bbox.h;

        // 转换为左上角和右下角坐标(相对于640x640输入图像)
        float x1 = cx - 0.5f * w;
        float y1 = cy - 0.5f * h;
        float x2 = cx + 0.5f * w;
        float y2 = cy + 0.5f * h;

        // 去除padding并缩放回原图尺寸
        x1 = std::max(0.0f, (x1 - pad_left) / scale);
        y1 = std::max(0.0f, (y1 - pad_top) / scale);
        x2 = std::min((float)image_w, (x2 - pad_left) / scale);
        y2 = std::min((float)image_h, (y2 - pad_top) / scale);

        // 转换回中心点坐标和宽高格式
        dets[i].bbox.x = (x1 + x2) / 2.0f; // 中心点x
        dets[i].bbox.y = (y1 + y2) / 2.0f; // 中心点y
        dets[i].bbox.w = x2 - x1;          // 宽度
        dets[i].bbox.h = y2 - y1;          // 高度

        LOGD("Det[%d]: input_bbox(%.1f,%.1f,%.1f,%.1f) -> "
             "output_bbox(%.1f,%.1f,%.1f,%.1f)",
             i, cx, cy, w, h, dets[i].bbox.x, dets[i].bbox.y, dets[i].bbox.w, dets[i].bbox.h);
    }
}

/**
 * @brief
 * @param output
 * @note scores_shape : [batch , class_num, det_num, 1]
 * @note des_shape: [batch, 1, 4, det_num]
 * @return int
 */
int getDetections(CVI_TENSOR* output, int32_t input_height, int32_t input_width, int classes_num,
                  CVI_SHAPE output_shape, float conf_thresh, std::vector<detection>& dets) {
    // 添加调试信息：打印输出tensor信息
    LOGD("=== DEBUG: Output tensor information ===");
    LOGD("Output shape: [%d, %d, %d, %d]", output_shape.dim[0], output_shape.dim[1], output_shape.dim[2],
           output_shape.dim[3]);

    // 检查是否有足够的输出tensor
    if (output == nullptr) {
        LOGE("ERROR: output tensor is null");
        return 0;
    }

    float* output_ptr = (float*)CVI_NN_TensorPtr(&output[0]);

    // 检查指针是否有效
    if (output_ptr == nullptr) {
        LOGE("ERROR: tensor pointer is null");
        return 0;
    }

    float stride[3] = {8, 16, 32};
    int count = 0;
    int batch = output_shape.dim[0];
    int channels = output_shape.dim[1];      // 应该是4(bbox) + 1(objectness) + classes_num
    int total_anchors = output_shape.dim[2]; // 27600

    LOGD("Batch: %d, Channels: %d, Total_anchors: %d", batch, channels, total_anchors);

    // 计算每个stride层的anchor数量
    int anchor_counts[3];
    for (int i = 0; i < 3; i++) {
        int nh = input_height / stride[i];
        int nw = input_width / stride[i];
        anchor_counts[i] = nh * nw;
        LOGD("Stride[%d]: %f, grid: %dx%d, anchors: %d", i, stride[i], nh, nw, anchor_counts[i]);
    }

    int anchor_offset = 0;
    for (int b = 0; b < batch; b++) {
        anchor_offset = 0;
        for (int stride_idx = 0; stride_idx < 3; stride_idx++) {
            int nh = input_height / stride[stride_idx];
            int nw = input_width / stride[stride_idx];
            int current_anchors = anchor_counts[stride_idx];

            for (int anchor_idx = 0; anchor_idx < current_anchors; anchor_idx++) {
                int total_anchor_idx = anchor_offset + anchor_idx;

                // 获取objectness/confidence (第5个通道)
                float objectness = output_ptr[4 * total_anchors + total_anchor_idx];

                // 添加调试输出：打印前几个anchor的原始数据
                if (objectness > 0.1 || total_anchor_idx < 3) { // Reduced debug spam condition
                    LOGD("Anchor[%d]: raw values = [%.6f, %.6f, %.6f, %.6f, %.6f]", total_anchor_idx,
                           output_ptr[0 * total_anchors + total_anchor_idx],
                           output_ptr[1 * total_anchors + total_anchor_idx],
                           output_ptr[2 * total_anchors + total_anchor_idx],
                           output_ptr[3 * total_anchors + total_anchor_idx],
                           output_ptr[4 * total_anchors + total_anchor_idx]);
                }

                if (objectness <= conf_thresh) {
                    continue;
                }

                // 获取bbox坐标 (前4个通道)
                // YOLOv8输出格式：cx, cy, w, h (相对于640x640输入图像的绝对像素坐标)
                float cx = output_ptr[0 * total_anchors + total_anchor_idx];
                float cy = output_ptr[1 * total_anchors + total_anchor_idx];
                float w = output_ptr[2 * total_anchors + total_anchor_idx];
                float h = output_ptr[3 * total_anchors + total_anchor_idx];

                detection det;
                det.score = objectness;
                det.cls = 0; // 单类别网球检测
                det.batch_idx = b;

                // 直接使用模型输出的中心点坐标和宽高，无需grid计算
                det.bbox.x = cx; // 中心点x坐标
                det.bbox.y = cy; // 中心点y坐标
                det.bbox.w = w;  // 宽度
                det.bbox.h = h;  // 高度

                LOGD("Detection[%d]: conf=%.3f, bbox_center=(%.1f,%.1f), "
                       "size=(%.1f,%.1f)",
                       count, objectness, cx, cy, w, h);

                count++;
                dets.emplace_back(det);
            }

            anchor_offset += current_anchors;
        }
    }
    return count;
}

// ============ 状态机 ============

enum RobotStatus {
    STATUS_CHASE_TENNIS,     // 追球：area < GRAB_AREA 且球可见
    STATUS_GRAB_TENNIS,      // 抓取：area >= GRAB_AREA，停车抓球
};

static const char* status_name(RobotStatus s) {
    switch (s) {
        case STATUS_CHASE_TENNIS:    return "chase";
        case STATUS_GRAB_TENNIS:     return "grab";
    }
    return "unknown";
}

// 控制参数
static const int FRAME_WIDTH       = 640;
static const float GRAB_AREA       = 0.40f;  // area_ratio >= 此值 → 抓取（提前触发抵消惯性）
static const int CENTER_MARGIN     = 35;     // 球中心距画面中心 ±35px 内算居中
static const int CHASE_SPEED       = 56;     // 追球前进速度
static const int TURN_SPEED        = 18;     // 原地转向速度（数值越小越慢）
static const int IDLE_SPEED        = 18;     // 没看到球时的搜索速度
static const float K_TURN_PULSE    = 5000.0f; // 转向脉冲系数(ms)：pulse = K * area_ratio
static const int TURN_PULSE_MIN    = 75 * 1000;  // 最小脉冲 75ms（保证电机能动）
static const int TURN_PULSE_MAX    = 500 * 1000;  // 最大脉冲 500ms（防止近处转过头丢球）
static const int GRAB_CONFIRM_THRESHOLD = 5;
static const float GRAB_AREA_MAX = 0.55f;  // area 超过此值太近，先后退
static const int BACKWARD_SPEED = 18;      // 后退速度
static const int BACKWARD_PULSE_US = 200 * 1000; // 后退脉冲时间
static const int GRAB_LEFT_TURN_SPEED = 18;       // 抓取前左转补偿速度（数值越小越慢）
static const int GRAB_LEFT_TURN_US    = 150 * 1000; // 抓取前左转补偿时间
static const int GRAB_LEFT_TURN_COUNT = 4;          // 左转补偿次数

struct RobotState {
    RobotStatus status;
    float area_ratio;
    int ball_cx;    // 球中心 x
    int grab_confirm_count;

    RobotState() : status(STATUS_CHASE_TENNIS), area_ratio(0), ball_cx(0), grab_confirm_count(0) {}
};




int main(int argc, char** argv) {
    int ret = 0;
    CVI_MODEL_HANDLE model;



    if (argc < 2 || argc > 3) {
        usage(argv);
        exit(-1);
    }

    CVI_U8 vi_channel = 0; // Default VI channel
    if (argc == 3) {
        vi_channel = atoi(argv[2]);
    }

    // 清理上次异常退出残留的硬件资源
    {
        VICapture cleanup_vi;
        cleanup_vi.deinit();  // 无操作或释放残留资源
    }

    // Initialize VI system
    VICapture vi_capture;
    g_vi_capture = &vi_capture;
    if (vi_capture.init() != CVI_SUCCESS) {
        exit(-1);
    }

    // Wait for sensor to stabilize
    usleep(500 * 1000);

#if USE_VPSS_RESIZE
    // Initialize VPSS for hardware resize (2560x1440 -> 640x640)
    LOGI("Initializing VPSS for hardware resize...");
    if (vi_capture.initVpssResize(2560, 1440, 640, 640) != CVI_SUCCESS) {
        LOGE("Failed to initialize VPSS");
        vi_capture.deinit();
        exit(-1);
    }
    LOGI("VPSS initialized successfully");
#endif

    // 初始化电机、机械臂和状态机
    Motor motor;
    g_motor = &motor;
    Arm arm;
    RobotState robot;
    arm.grab_pos(); // 机械臂回到待抓取位置
    CVI_TENSOR* input;
    CVI_TENSOR* output;
    CVI_TENSOR* input_tensors;
    CVI_TENSOR* output_tensors;
    int32_t input_num;
    int32_t output_num;
    CVI_SHAPE input_shape;
    CVI_SHAPE* output_shape;
    int32_t height;
    int32_t width;
    // int bbox_len = 5; // 1 class + 4 bbox
    int classes_num = 1;
    float conf_thresh = 0.5;
    float iou_thresh = 0.5;
    ret = CVI_NN_RegisterModel(argv[1], &model);
    g_model = model;
    if (ret != CVI_RC_SUCCESS) {
        exit(1);
    }
    LOGI("CVI_NN_RegisterModel succeeded");

    // get input output tensors
    CVI_NN_GetInputOutputTensors(model, &input_tensors, &input_num, &output_tensors, &output_num);

    LOGI("=== Model information ===");
    LOGI("Input number: %d, Output number: %d", input_num, output_num);

    input = CVI_NN_GetTensorByName(CVI_NN_DEFAULT_TENSOR, input_tensors, input_num);
    assert(input);
    output = output_tensors;
    output_shape = reinterpret_cast<CVI_SHAPE*>(calloc(output_num, sizeof(CVI_SHAPE)));
    for (int i = 0; i < output_num; i++) {
        output_shape[i] = CVI_NN_TensorShape(&output[i]);
        LOGI("Output[%d] shape: [%d, %d, %d, %d]", i, output_shape[i].dim[0], output_shape[i].dim[1],
               output_shape[i].dim[2], output_shape[i].dim[3]);
    }

    // nchw
    input_shape = CVI_NN_TensorShape(input);
    height = input_shape.dim[2];
    width = input_shape.dim[3];
    assert(height % 32 == 0 && width % 32 == 0);

    // 循环处理摄像头帧
    int frame_idx = 0;
    struct timeval start_time, end_time;
    struct timeval t1, t2;
    long total_time_us = 0;
    int frame_count = 0;

    cv::setNumThreads(1);

    // 注册信号处理，确保 Ctrl-C 时清理硬件资源
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    while (true) {
        gettimeofday(&start_time, NULL);

        frame_idx++;
        LOGI("\n[Frame %d]", frame_idx);

        // Get YUV frame from VI and convert to BGR
        gettimeofday(&t1, NULL);
        cv::Mat image;
        if (vi_capture.getFrameAsBGR(vi_channel, image) != CVI_SUCCESS) {
            LOGW("Failed to get frame from VI channel %d", vi_channel);
            usleep(100000); // 休眠0.1秒
            continue;
        }

        if (!image.data) {
            LOGW("Empty image data");
            usleep(100000);
            continue;
        }

        cv::Mat cloned = image.clone();
        gettimeofday(&t2, NULL);
        long read_time = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);

        // Image is already 640x640 from vi_get_frame_as_bgr, no need to resize again
        gettimeofday(&t1, NULL);
        
        // Convert BGR to RGB
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

        // Packed2Planar
        cv::Mat channels[3];
        for (int i = 0; i < 3; i++) {
            channels[i] = cv::Mat(image.rows, image.cols, CV_8SC1);
        }
        cv::split(image, channels);

        // fill data
        int8_t* ptr = (int8_t*)CVI_NN_TensorPtr(input);
        int channel_size = height * width;
        for (int i = 0; i < 3; ++i) {
            memcpy(ptr + i * channel_size, channels[i].data, channel_size);
        }
        gettimeofday(&t2, NULL);
        long preprocess_time = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);

        // run inference
        gettimeofday(&t1, NULL);
        CVI_NN_Forward(model, input_tensors, input_num, output_tensors, output_num);
        gettimeofday(&t2, NULL);
        long inference_time = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);
        // do post proprocess
        gettimeofday(&t1, NULL);
        int det_num = 0;
        std::vector<detection> dets;

        det_num = getDetections(output, height, width, classes_num, output_shape[0], conf_thresh, dets);
        // correct box with origin image size
        NMS(dets, &det_num, iou_thresh);
        correctYoloBoxes(dets, det_num, cloned.rows, cloned.cols, height, width);
        gettimeofday(&t2, NULL);
        long postprocess_time = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);
        // ============ 状态机控制逻辑 ============
        if (det_num > 0) {
            // 选择最大的球（最近的）
            int best_idx = 0;
            for (int i = 1; i < det_num; i++) {
                if (dets[i].bbox.w * dets[i].bbox.h > dets[best_idx].bbox.w * dets[best_idx].bbox.h)
                    best_idx = i;
            }

            box b = dets[best_idx].bbox;
            float img_area = cloned.cols * cloned.rows;
            float area_ratio = (b.w * b.h) / img_area;
            int ball_cx = static_cast<int>(b.x);
            int center = FRAME_WIDTH / 2;

            robot.area_ratio = area_ratio;
            robot.ball_cx = ball_cx;

            LOGI("[DETECT] area=%.3f cx=%d conf=%.3f status=%s",
                 area_ratio, ball_cx, dets[best_idx].score, status_name(robot.status));

            int offset = ball_cx - center;
            bool centered = abs(offset) <= CENTER_MARGIN;

            // 动态脉冲时间：area 越大脉冲越长，避免远距离转过头
            int pulse_us = std::max(TURN_PULSE_MIN, std::min(TURN_PULSE_MAX, (int)(K_TURN_PULSE * area_ratio * 1000)));

            if (area_ratio >= GRAB_AREA && centered) {
                // 球足够近且居中 → 确认计数
                robot.grab_confirm_count++;
                LOGI("[GRAB] confirm %d/%d (area=%.3f cx=%d)", robot.grab_confirm_count, GRAB_CONFIRM_THRESHOLD, area_ratio, ball_cx);

                if (area_ratio >= GRAB_AREA_MAX) {
                    // 太近了，微微后退
                    LOGI("[GRAB] Too close (area=%.3f > %.3f), backing up", area_ratio, GRAB_AREA_MAX);
                    motor.backward(BACKWARD_SPEED);
                    usleep(BACKWARD_PULSE_US);
                    motor.standby();
                } else {
                    motor.standby();
                }

                if (robot.grab_confirm_count >= GRAB_CONFIRM_THRESHOLD) {
                    // 太近后退一步再抓
                    if (area_ratio >= GRAB_AREA_MAX) {
                        LOGI("[GRAB] Backing up before grab");
                        motor.backward(BACKWARD_SPEED);
                        usleep(BACKWARD_PULSE_US);
                        motor.standby();
                    }

                    LOGI("[ARM] Confirmed, compensating gripper offset...");

                    // 爪子偏右，连续多次左转补偿
                    for (int i = 0; i < GRAB_LEFT_TURN_COUNT; i++) {
                        LOGI("[GRAB] Left turn compensation %d/%d (%dms)", i + 1, GRAB_LEFT_TURN_COUNT, GRAB_LEFT_TURN_US / 1000);
                        motor.drive(-GRAB_LEFT_TURN_SPEED, GRAB_LEFT_TURN_SPEED);
                        usleep(GRAB_LEFT_TURN_US);
                        motor.standby();
                        usleep(100 * 1000);  // 间隔100ms
                    }

                    LOGI("[ARM] Full grab sequence");

                    // 抓取: 伸下→夹紧→抬起→停2秒
                    arm.grab();
                    usleep(2000 * 1000);

                    // 松开→复位
                    arm.release();
                    usleep(1000 * 1000);
                    arm.grab_pos();
                    usleep(1000 * 1000);
                    robot.grab_confirm_count = 0;
                    robot.status = STATUS_CHASE_TENNIS;
                }
            } else if (area_ratio >= GRAB_AREA && !centered) {
                // 球够近但没居中 → 原地微调（脉冲式），不前进
                robot.grab_confirm_count = 0;
                LOGI("[ALIGN] area=%.3f OK but cx=%d not centered, pulse=%dms", area_ratio, ball_cx, pulse_us / 1000);
                if (offset < 0) {
                    motor.drive(-TURN_SPEED, TURN_SPEED);
                } else {
                    motor.drive(TURN_SPEED, -TURN_SPEED);
                }
                usleep(pulse_us);
                motor.standby();
            } else {
                // 追球
                robot.grab_confirm_count = 0;
                robot.status = STATUS_CHASE_TENNIS;

                if (!centered) {
                    // 球偏左或偏右 → 差速原地转向（脉冲式）
                    if (offset < 0) {
                        LOGI("[MOTOR] TURN LEFT (cx=%d offset=%d pulse=%dms)", ball_cx, offset, pulse_us / 1000);
                        motor.drive(-TURN_SPEED, TURN_SPEED);
                    } else {
                        LOGI("[MOTOR] TURN RIGHT (cx=%d offset=%d pulse=%dms)", ball_cx, offset, pulse_us / 1000);
                        motor.drive(TURN_SPEED, -TURN_SPEED);
                    }
                    usleep(pulse_us);
                    motor.standby();
                } else {
                    // 球基本居中 → 前进
                    LOGI("[MOTOR] FORWARD (cx=%d area=%.3f < %.3f)", ball_cx, area_ratio, GRAB_AREA);
                    motor.forward(CHASE_SPEED);
                }
            }

#if ENABLE_DRAW_BBOX
            for (int i = 0; i < det_num; i++) {
                box b = dets[i].bbox;
                int x1 = std::max(0, (int)(b.x - b.w / 2));
                int y1 = std::max(0, (int)(b.y - b.h / 2));
                int x2 = std::min(cloned.cols - 1, (int)(b.x + b.w / 2));
                int y2 = std::min(cloned.rows - 1, (int)(b.y + b.h / 2));
                cv::rectangle(cloned, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 255), 3, 8, 0);
                float ar = (b.w * b.h) / img_area;
                char content[100];
                sprintf(content, "tennis %.3f area:%.3f", dets[i].score, ar);
                cv::putText(cloned, content, cv::Point(x1, y1 - 10), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            }

            // Center line (red)
            cv::line(cloned, cv::Point(center, 0), cv::Point(center, cloned.rows), cv::Scalar(0, 0, 255), 1);
            // Center margin lines (green)
            cv::line(cloned, cv::Point(center - CENTER_MARGIN, 0), cv::Point(center - CENTER_MARGIN, cloned.rows), cv::Scalar(0, 255, 0), 1);
            cv::line(cloned, cv::Point(center + CENTER_MARGIN, 0), cv::Point(center + CENTER_MARGIN, cloned.rows), cv::Scalar(0, 255, 0), 1);

            // Status info
            char info[128];
            sprintf(info, "status=%s area=%.3f cx=%d grab_area=%.2f",
                    status_name(robot.status), area_ratio, ball_cx, GRAB_AREA);
            cv::putText(cloned, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
#endif

        } else {
            LOGI("[DETECT] No ball detected, searching...");
            robot.grab_confirm_count = 0;
            robot.status = STATUS_CHASE_TENNIS;
            motor.drive(IDLE_SPEED, -IDLE_SPEED);  // 没看到球就连续右转搜索
        }
        
#if ENABLE_SAVE_IMAGE
            {
                const char* save_dir = "/root/images";
                mkdir(save_dir, 0755);  // ensure directory exists
                char output_path[256];
                sprintf(output_path, "%s/detected_%d.jpg", save_dir, frame_idx);
                bool ok = cv::imwrite(output_path, cloned);
                LOGI("[SAVE] %s %s", output_path, ok ? "OK" : "FAILED");
            }
#endif
        // 计算帧率
        gettimeofday(&end_time, NULL);
        long frame_time_us = (end_time.tv_sec - start_time.tv_sec) * 1000000 + (end_time.tv_usec - start_time.tv_usec);
        float fps = 1000000.0f / frame_time_us;

        frame_count++;
        total_time_us += frame_time_us;
        float avg_fps = 1000000.0f * frame_count / total_time_us;

        LOGI("[FPS] %.2f  avg: %.2f  (%.1fms)", fps, avg_fps, frame_time_us / 1000.0f);

        // 持续运行，无帧数限制
        // if (frame_idx >= 200) break;
    } // end while loop

    // Cleanup
    cleanup_and_exit();
    free(output_shape);
    return 0;
}