// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka0-ref commits 64f8dab, 7b7011d
// Description: 通用 YOLO 检测模块实现 - 支持所有模型格式（INT8/UINT8/BF16/FP32/INT16）

#include "detect.hpp"
#include <stdio.h>
#include <math.h>
#include  <string.h>
#include <algorithm>
#include "cviruntime.h"

// =====================================================
// 辅助函数
// =====================================================

template <typename T>
int argmax(const T* data, size_t len, size_t stride = 1) {
    int maxIndex = 0;
    for (size_t i = stride; i < len; i += stride) {
        if (data[maxIndex] < data[i]) {
            maxIndex = i;
        }
    }
    return maxIndex;
}

// =====================================================
// 检测函数实现
// =====================================================

float calIou(box a, box b) {
    float area1 = a.w * a.h;
    float area2 = b.w * b.h;
    float wi = std::min((a.x + a.w / 2), (b.x + b.w / 2)) - std::max((a.x - a.w / 2), (b.x - b.w / 2));
    float hi = std::min((a.y + a.h / 2), (b.y + b.h / 2)) - std::max((a.y - a.h / 2), (b.y - b.h / 2));
    float area_i = std::max(wi, 0.0f) * std::max(hi, 0.0f);
    return area_i / (area1 + area2 - area_i);
}

void NMS(std::vector<detection>& dets, int* total, float thresh) {
    if (*total) {
        std::sort(dets.begin(), dets.end(), [](detection& a, detection& b) { return b.score < a.score; });
        int new_count = *total;
        for (int i = 0; i < *total; ++i) {
            detection& a = dets[i];
            if (a.score == 0) continue;
            for (int j = i + 1; j < *total; ++j) {
                detection& b = dets[j];
                if (dets[i].batch_idx == dets[j].batch_idx &&
                    b.score != 0 && dets[i].cls == dets[j].cls &&
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

void correctYoloBoxes(std::vector<detection>& dets, int det_num,
                      int image_h, int image_w,
                      int input_height, int input_width) {
    float scale = std::min((float)input_width / image_w, (float)input_height / image_h);
    int new_h = (int)(image_h * scale);
    int new_w = (int)(image_w * scale);
    int pad_top = (input_height - new_h) / 2;
    int pad_left = (input_width - new_w) / 2;

    for (int i = 0; i < det_num; ++i) {
        float cx = dets[i].bbox.x;
        float cy = dets[i].bbox.y;
        float w  = dets[i].bbox.w;
        float h  = dets[i].bbox.h;

        float x1 = cx - 0.5f * w;
        float y1 = cy - 0.5f * h;
        float x2 = cx + 0.5f * w;
        float y2 = cy + 0.5f * h;

        x1 = std::max(0.0f, (x1 - pad_left) / scale);
        y1 = std::max(0.0f, (y1 - pad_top) / scale);
        x2 = std::min((float)image_w, (x2 - pad_left) / scale);
        y2 = std::min((float)image_h, (y2 - pad_top) / scale);

        dets[i].bbox.x = (x1 + x2) / 2.0f;
        dets[i].bbox.y = (y1 + y2) / 2.0f;
        dets[i].bbox.w = x2 - x1;
        dets[i].bbox.h = y2 - y1;
    }
}

/**
 * @brief Parse single fused output tensor from YOLOv8
 * @note output shape: [batch, 4+classes, num_boxes, 1]
 *       layout: [cx, cy, w, h, score0, score1, ...] x num_boxes
 *       coords are in input image pixel space (already decoded)
 * @note 支持所有模型格式：INT8/UINT8/BF16/FP32/INT16
 */
int getDetections(CVI_TENSOR* output,
                  int32_t input_height,
                  int32_t input_width,
                  int classes_num,
                  CVI_SHAPE output_shape,
                  float conf_thresh,
                  std::vector<detection>& dets) {

    // output[0] is the only tensor: [batch, 4+classes, num_boxes, 1]
    int batch     = output_shape.dim[0];
    int channels  = output_shape.dim[1]; // 4 + classes_num
    int num_boxes = output_shape.dim[2];

    printf("[DETECT] batch=%d channels=%d num_boxes=%d fmt=%d qscale=%f\n",
           batch, channels, num_boxes, (int)output[0].fmt, output[0].qscale);

    // 反量化到 float
    size_t count = output[0].count;
    float* data;
    bool allocated = false;

    if (output[0].fmt == CVI_FMT_FP32) {
        // FP32 格式：直接使用指针
        data = (float*)CVI_NN_TensorPtr(&output[0]);
    } else {
        // 其他格式：需要反量化
        data = (float*)malloc(count * sizeof(float));
        allocated = true;
        float qscale = output[0].qscale;
        void* src_ptr = CVI_NN_TensorPtr(&output[0]);  // synced pointer

        if (output[0].fmt == CVI_FMT_INT8) {
            // INT8 反量化
            int8_t* src = (int8_t*)src_ptr;
            for (size_t i = 0; i < count; i++) {
                data[i] = src[i] * qscale;
            }
            printf("[DETECT] INT8 model detected, dequantized with qscale=%.6f\n", qscale);
        } else if (output[0].fmt == CVI_FMT_UINT8) {
            // UINT8 反量化
            uint8_t* src = (uint8_t*)src_ptr;
            for (size_t i = 0; i < count; i++) {
                data[i] = (src[i] - output[0].zero_point) * qscale;
            }
            printf("[DETECT] UINT8 model detected, dequantized with qscale=%.6f zp=%d\n",
                   qscale, output[0].zero_point);
        } else if (output[0].fmt == CVI_FMT_BF16) {
            // BF16 转 FP32
            uint16_t* src = (uint16_t*)src_ptr;
            for (size_t i = 0; i < count; i++) {
                uint32_t v = (uint32_t)src[i] << 16;
                memcpy(&data[i], &v, sizeof(float));
            }
            printf("[DETECT] BF16 model detected, converted to FP32\n");
        } else if (output[0].fmt == CVI_FMT_INT16) {
            // INT16 反量化
            int16_t* src = (int16_t*)src_ptr;
            for (size_t i = 0; i < count; i++) {
                data[i] = src[i] * qscale;
            }
            printf("[DETECT] INT16 model detected, dequantized with qscale=%.6f\n", qscale);
        } else {
            printf("[DETECT] Unsupported format: %d\n", (int)output[0].fmt);
            memset(data, 0, count * sizeof(float));
        }
    }

    int det_count = 0;
    for (int b = 0; b < batch; b++) {
        // base pointer for this batch: [channels, num_boxes]
        float* base = data + b * channels * num_boxes;
        float* cx_row = base + 0 * num_boxes;
        float* cy_row = base + 1 * num_boxes;
        float* w_row  = base + 2 * num_boxes;
        float* h_row  = base + 3 * num_boxes;

        for (int j = 0; j < num_boxes; j++) {
            // 模型输出已经是 sigmoid 后的值，直接使用
            float max_score = -1.0f;
            int   max_cls   = 0;
            for (int c = 0; c < classes_num; c++) {
                float s = base[(4 + c) * num_boxes + j];
                if (s > max_score) { max_score = s; max_cls = c; }
            }
            if (max_score <= conf_thresh) continue;

            detection det;
            det.score     = max_score;
            det.cls       = max_cls;
            det.batch_idx = b;
            det.bbox.x    = cx_row[j];
            det.bbox.y    = cy_row[j];
            det.bbox.w    = w_row[j];
            det.bbox.h    = h_row[j];
            dets.emplace_back(det);
            det_count++;
        }
    }

    if (allocated) free(data);
    return det_count;
}
