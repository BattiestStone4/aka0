// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka0-ref commit 64f8dab
// Description: 静态图片YOLO检测工具，用于验证模型效果

#include <stdio.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include "cviruntime.h"
#include "detect.hpp"

static const char *class_names[] = { "tennis ball" };

static void usage(char **argv) {
  printf("Usage:\n");
  printf("   %s cvimodel image.jpg image_detected.jpg\n", argv[0]);
}

int main(int argc, char **argv) {
  int ret = 0;
  CVI_MODEL_HANDLE model;

  if (argc != 4) {
    usage(argv);
    exit(-1);
  }
  CVI_TENSOR *input;
  CVI_TENSOR *output;
  CVI_TENSOR *input_tensors;
  CVI_TENSOR *output_tensors;
  int32_t input_num;
  int32_t output_num;
  CVI_SHAPE input_shape;
  CVI_SHAPE* output_shape;
  int32_t height;
  int32_t width;
  int classes_num = 1;
  float conf_thresh = 0.5;
  float iou_thresh = 0.5;

  ret = CVI_NN_RegisterModel(argv[1], &model);
  if (ret != CVI_RC_SUCCESS) {
    printf("CVI_NN_RegisterModel failed, err %d\n", ret);
    exit(1);
  }
  printf("CVI_NN_RegisterModel succeeded\n");

  // get input output tensors
  CVI_NN_GetInputOutputTensors(model, &input_tensors, &input_num, &output_tensors,
                               &output_num);

  input = CVI_NN_GetTensorByName(CVI_NN_DEFAULT_TENSOR, input_tensors, input_num);
  assert(input);
  output = output_tensors;
  printf("debug: output_num=%d\n", output_num);
  output_shape = reinterpret_cast<CVI_SHAPE *>(calloc(output_num, sizeof(CVI_SHAPE)));
  for (int i = 0; i < output_num; i++)
  {
    output_shape[i] = CVI_NN_TensorShape(&output[i]);
    printf("debug: output[%d] shape=[%d,%d,%d,%d] fmt=%d count=%zu qscale=%f\n",
           i, output_shape[i].dim[0], output_shape[i].dim[1],
           output_shape[i].dim[2], output_shape[i].dim[3],
           (int)output[i].fmt, output[i].count, output[i].qscale);
  }

  // nchw
  input_shape = CVI_NN_TensorShape(input);
  height = input_shape.dim[2];
  width = input_shape.dim[3];
  assert(height % 32 == 0 && width %32 == 0);

  // imread
  cv::Mat image;
  image = cv::imread(argv[2]);
  if (!image.data) {
    printf("Could not open or find the image\n");
    return -1;
  }
  cv::Mat cloned = image.clone();

  // resize & letterbox
  int ih = image.rows;
  int iw = image.cols;
  int oh = height;
  int ow = width;
  double resize_scale = std::min((double)oh / ih, (double)ow / iw);
  int nh = (int)(ih * resize_scale);
  int nw = (int)(iw * resize_scale);
  cv::resize(image, image, cv::Size(nw, nh));
  int top = (oh - nh) / 2;
  int bottom = (oh - nh) - top;
  int left = (ow - nw) / 2;
  int right = (ow - nw) - left;
  cv::copyMakeBorder(image, image, top, bottom, left, right, cv::BORDER_CONSTANT,
                     cv::Scalar::all(0));
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

  // Fill input tensor based on its format
  printf("debug: input fmt=%d qscale=%f zero_point=%d\n", (int)input->fmt, input->qscale, input->zero_point);
  int channel_size = height * width;

  if (input->fmt == CVI_FMT_FP32) {
    // TPU-MLIR bakes normalization into the model, feed raw [0,255] as float
    cv::Mat channels_f[3];
    cv::Mat image_f;
    image.convertTo(image_f, CV_32FC3, 1.0);
    cv::split(image_f, channels_f);
    float *ptr = (float *)CVI_NN_TensorPtr(input);
    for (int i = 0; i < 3; ++i) {
      memcpy(ptr + i * channel_size, channels_f[i].data, channel_size * sizeof(float));
    }
  } else {
    // INT8/UINT8 fused preprocess: copy raw bytes
    cv::Mat channels_b[3];
    for (int i = 0; i < 3; i++) {
      channels_b[i] = cv::Mat(image.rows, image.cols, CV_8UC1);
    }
    cv::split(image, channels_b);
    uint8_t *ptr = (uint8_t *)CVI_NN_TensorPtr(input);
    for (int i = 0; i < 3; ++i) {
      memcpy(ptr + i * channel_size, channels_b[i].data, channel_size);
    }
  }

  // run inference
  struct timeval t0, t1;
  gettimeofday(&t0, NULL);
  CVI_NN_Forward(model, input_tensors, input_num, output_tensors, output_num);
  gettimeofday(&t1, NULL);
  long elapsed_ms = (t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_usec - t0.tv_usec) / 1000;
  printf("CVI_NN_Forward Succeed... elapsed: %ld ms\n", elapsed_ms);

  // do post proprocess
  int det_num = 0;
  std::vector<detection> dets;
  det_num = getDetections(output, height, width, classes_num, output_shape[0],
                          conf_thresh, dets);
  // correct box with origin image size
  NMS(dets, &det_num, iou_thresh);
  // bbox is in input_image space (cx,cy,w,h), map back to original image
  correctYoloBoxes(dets, det_num, cloned.rows, cloned.cols, height, width);

  // draw bbox on image
  for (int i = 0; i < det_num; i++) {
    box b = dets[i].bbox;
    // xywh2xyxy
    int x1 = (b.x - b.w / 2);
    int y1 = (b.y - b.h / 2);
    int x2 = (b.x + b.w / 2);
    int y2 = (b.y + b.h / 2);
    cv::rectangle(cloned, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 0),
                  3, 8, 0);
    char content[100];
    sprintf(content, "%s %0.3f", class_names[dets[i].cls], dets[i].score);
    cv::putText(cloned, content, cv::Point(x1, y1),
                cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
  }

  // save or show picture
  cv::imwrite(argv[3], cloned);

  printf("------\n");
  printf("%d objects are detected\n", det_num);
  printf("------\n");

  CVI_NN_CleanupModel(model);
  printf("CVI_NN_CleanupModel succeeded\n");
  free(output_shape);
  return 0;
}
