// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka0-ref commits 64f8dab, 7b7011d
// Description: 通用 YOLO 检测模块 - 支持所有模型格式（INT8/UINT8/BF16/FP32/INT16）
//
// 此模块提供：
// - 通用模型输出反量化（支持所有 CVI_FMT_* 格式）
// - YOLOv8 检测后处理（NMS, 坐标校正）
// - 可被 tennis.cpp, detect2.cpp, test_capture_detect.cpp 共用

#ifndef DETECT_HPP
#define DETECT_HPP

#include <vector>
#include "cviruntime.h"

// 检测框结构
typedef struct {
    float x, y, w, h;
} box;

// 检测结果结构
typedef struct {
    box bbox;
    int cls;
    float score;
    int batch_idx;
} detection;

// =====================================================
// 检测函数接口
// =====================================================

// 计算 IOU（用于 NMS）
float calIou(box a, box b);

// 非极大值抑制
void NMS(std::vector<detection>& dets, int* total, float thresh);

// YOLO 坐标校正（从输入图像空间映射到原图）
void correctYoloBoxes(std::vector<detection>& dets, int det_num,
                      int image_h, int image_w,
                      int input_height, int input_width);

// 通用检测函数（支持所有模型格式）
// 返回检测到的目标数量
// 参数:
//   output - 模型输出 tensor
//   input_height/width - 模型输入尺寸
//   classes_num - 类别数量
//   output_shape - 输出 tensor 形状
//   conf_thresh - 置信度阈值
//   dets - 输出检测结果列表
int getDetections(CVI_TENSOR* output,
                  int32_t input_height,
                  int32_t input_width,
                  int classes_num,
                  CVI_SHAPE output_shape,
                  float conf_thresh,
                  std::vector<detection>& dets);

#endif // DETECT_HPP
