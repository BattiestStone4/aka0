// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka0-ref commit 64f8dab
// Description: 单帧拍照+YOLO检测工具，用于调试相机和检测算法

#include <opencv2/opencv.hpp>
#include "vi_capture.hpp"
#include "cviruntime.h"
#include "logger.hpp"
#include <unistd.h>
#include <sys/stat.h>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>

typedef struct {
    float x, y, w, h;
} box;

typedef struct {
    box bbox;
    int cls;
    float score;
    int batch_idx;
} detection;

// ---- YOLO post-processing (same as tennis.cpp) ----

float calIou(box a, box b) {
    float area1 = a.w * a.h;
    float area2 = b.w * a.h;
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
            if (dets[i].score == 0) continue;
            for (int j = i + 1; j < *total; ++j) {
                if (dets[j].score != 0 && dets[i].cls == dets[j].cls &&
                    calIou(dets[i].bbox, dets[j].bbox) > thresh) {
                    dets[j].score = 0;
                    new_count--;
                }
            }
        }
        dets.erase(std::remove_if(dets.begin(), dets.end(),
            [](const detection& d) { return d.score == 0; }), dets.end());
        *total = new_count;
    }
}

void correctYoloBoxes(std::vector<detection>& dets, int det_num, int image_h, int image_w,
                      int input_height, int input_width) {
    float scale = std::min((float)input_width / image_w, (float)input_height / image_h);
    int new_h = (int)(image_h * scale);
    int new_w = (int)(image_w * scale);
    int pad_top = (input_height - new_h) / 2;
    int pad_left = (input_width - new_w) / 2;
    for (int i = 0; i < det_num; ++i) {
        float cx = dets[i].bbox.x, cy = dets[i].bbox.y;
        float w = dets[i].bbox.w, h = dets[i].bbox.h;
        float x1 = std::max(0.0f, (cx - 0.5f * w - pad_left) / scale);
        float y1 = std::max(0.0f, (cy - 0.5f * h - pad_top) / scale);
        float x2 = std::min((float)image_w, (cx + 0.5f * w - pad_left) / scale);
        float y2 = std::min((float)image_h, (cy + 0.5f * h - pad_top) / scale);
        dets[i].bbox.x = (x1 + x2) / 2.0f;
        dets[i].bbox.y = (y1 + y2) / 2.0f;
        dets[i].bbox.w = x2 - x1;
        dets[i].bbox.h = y2 - y1;
    }
}

int getDetections(CVI_TENSOR* output, int32_t input_height, int32_t input_width,
                  CVI_SHAPE output_shape, float conf_thresh, std::vector<detection>& dets) {
    if (!output) return 0;
    float* ptr = (float*)CVI_NN_TensorPtr(&output[0]);
    if (!ptr) return 0;

    float stride[3] = {8, 16, 32};
    int batch = output_shape.dim[0];
    int channels = output_shape.dim[1];
    int total_anchors = output_shape.dim[2];
    int count = 0, anchor_offset = 0;

    for (int b = 0; b < batch; b++) {
        anchor_offset = 0;
        for (int s = 0; s < 3; s++) {
            int nh = input_height / (int)stride[s];
            int nw = input_width / (int)stride[s];
            int current = nh * nw;
            for (int a = 0; a < current; a++) {
                int idx = anchor_offset + a;
                float obj = ptr[4 * total_anchors + idx];
                if (obj <= conf_thresh) continue;
                detection det;
                det.score = obj;
                det.cls = 0;
                det.batch_idx = b;
                det.bbox.x = ptr[0 * total_anchors + idx];
                det.bbox.y = ptr[1 * total_anchors + idx];
                det.bbox.w = ptr[2 * total_anchors + idx];
                det.bbox.h = ptr[3 * total_anchors + idx];
                count++;
                dets.emplace_back(det);
            }
            anchor_offset += current;
        }
    }
    return count;
}

// ---- Main ----

int main(int argc, char** argv) {
    if (argc < 2) {
        LOGI("Usage: %s model.cvimodel [output.jpg]", argv[0]);
        return 1;
    }
    const char* model_path = argv[1];
    const char* output_path = "/root/images/capture.jpg";
    if (argc > 2) output_path = argv[2];

    // Ensure output directory
    {
        std::string path(output_path);
        size_t pos = path.rfind('/');
        if (pos != std::string::npos) mkdir(path.substr(0, pos).c_str(), 0755);
    }

    // Init VI
    VICapture vi;
    if (vi.init() != CVI_SUCCESS) { LOGE("VI init failed"); return 1; }
#if USE_VPSS_RESIZE
    if (vi.initVpssResize(2560, 1440, 640, 640) != CVI_SUCCESS) { LOGE("VPSS init failed"); vi.deinit(); return 1; }
#endif

    // Wait for sensor to stabilize, then discard first few frames
    LOGI("Warming up sensor...");
    usleep(1000 * 1000);  // 1 second
    cv::Mat dummy;
    for (int i = 0; i < 10; i++) {
        vi.getFrameAsBGR(0, dummy);
        usleep(50000);  // 50ms between frames
    }

    // Capture the real frame
    cv::Mat image;
    if (vi.getFrameAsBGR(0, image) != CVI_SUCCESS || !image.data) {
        LOGE("Failed to capture frame");
        vi.deinit();
        return 1;
    }
    cv::Mat drawn = image.clone();
    LOGI("Captured: %dx%d", image.cols, image.rows);

#if USE_VPSS_RESIZE
    vi.deinitVpssResize();
#endif
    vi.deinit();

    // Init model
    CVI_MODEL_HANDLE model;
    if (CVI_NN_RegisterModel(model_path, &model) != CVI_RC_SUCCESS) {
        LOGE("Failed to load model");
        return 1;
    }
    CVI_TENSOR *input_tensors, *output_tensors;
    int32_t input_num, output_num;
    CVI_NN_GetInputOutputTensors(model, &input_tensors, &input_num, &output_tensors, &output_num);

    CVI_TENSOR* input = CVI_NN_GetTensorByName(CVI_NN_DEFAULT_TENSOR, input_tensors, input_num);
    CVI_SHAPE input_shape = CVI_NN_TensorShape(input);
    int height = input_shape.dim[2];
    int width = input_shape.dim[3];

    // Prepare output shapes
    CVI_SHAPE* output_shape = (CVI_SHAPE*)calloc(output_num, sizeof(CVI_SHAPE));
    for (int i = 0; i < output_num; i++)
        output_shape[i] = CVI_NN_TensorShape(&output_tensors[i]);

    // Preprocess: BGR -> RGB -> split -> fill tensor
    cv::Mat rgb;
    cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
    cv::Mat channels[3];
    for (int i = 0; i < 3; i++)
        channels[i] = cv::Mat(rgb.rows, rgb.cols, CV_8SC1);
    cv::split(rgb, channels);
    int8_t* iptr = (int8_t*)CVI_NN_TensorPtr(input);
    int channel_size = height * width;
    for (int i = 0; i < 3; ++i)
        memcpy(iptr + i * channel_size, channels[i].data, channel_size);

    // Inference
    CVI_NN_Forward(model, input_tensors, input_num, output_tensors, output_num);

    // Post-process
    float conf_thresh = 0.5f, iou_thresh = 0.5f;
    std::vector<detection> dets;
    int det_num = getDetections(output_tensors, height, width, output_shape[0], conf_thresh, dets);
    NMS(dets, &det_num, iou_thresh);
    correctYoloBoxes(dets, det_num, image.rows, image.cols, height, width);

    float img_area = image.cols * image.rows;

    if (det_num > 0) {
        for (int i = 0; i < det_num; i++) {
            box b = dets[i].bbox;
            int x1 = std::max(0, (int)(b.x - b.w / 2));
            int y1 = std::max(0, (int)(b.y - b.h / 2));
            int x2 = std::min(image.cols - 1, (int)(b.x + b.w / 2));
            int y2 = std::min(image.rows - 1, (int)(b.y + b.h / 2));

            cv::rectangle(drawn, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 255), 3);

            float area_ratio = (b.w * b.h) / img_area;
            char text[128];
            sprintf(text, "tennis %.3f area:%.4f", dets[i].score, area_ratio);
            cv::putText(drawn, text, cv::Point(x1, y1 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

            LOGI("Det[%d]: conf=%.3f x=%.0f y=%.0f w=%.0f h=%.0f area_ratio=%.4f",
                 i, dets[i].score, b.x, b.y, b.w, b.h, area_ratio);
        }
    } else {
        LOGI("No tennis ball detected");
        cv::putText(drawn, "No detection", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }

    // Save
    if (cv::imwrite(output_path, drawn)) {
        LOGI("Saved: %s", output_path);
    } else {
        LOGE("Failed to save: %s", output_path);
    }

    CVI_NN_CleanupModel(model);
    free(output_shape);
    return 0;
}
