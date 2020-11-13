#ifndef COTEK_VISUAL_INCLUDE_COTEK_VISUAL_COTEK_VISUAL_OPTIONS_H_
#define COTEK_VISUAL_INCLUDE_COTEK_VISUAL_COTEK_VISUAL_OPTIONS_H_

#include <opencv2/opencv.hpp>
#include <vector>

// 视觉参数
struct ROI_ {
  int lt_x;
  int lt_y;
  int width;
  int height;
};

struct ROI_TYPE_ {
  ROI_ ROI_LEFT_UP_;
  ROI_ ROI_RIGHT_UP_;
  ROI_ ROI_STRAIGHT_UP_;
  ROI_ ROI_LEFT_DOWN_;
  ROI_ ROI_RIGHT_DOWN_;
  ROI_ ROI_STRAIGHT_DOWN_;
};

struct D435_basic_config_ {
  std::vector<int> arethread;
  std::vector<double> up_to_nums;
  int up_num;
  int nums;
  int threshold_x;
  int threshold_y;
  float left_delta_of_D435;
  double up_pile;
  bool enable_visual_debug;
};

struct ROI_Factory {
  cv::Rect ROI_STRAIGHT_UP_;
  cv::Rect ROI_LEFT_UP_;
  cv::Rect ROI_RIGHT_UP_;
  cv::Rect ROI_STRAIGHT_DOWN_;
  cv::Rect ROI_LEFT_DOWN_;
  cv::Rect ROI_RIGHT_DOWN_;
};
struct RealSenseOption {
  // 视觉基本配置
//   ROI_TYPE_ ROI_TYPES;
  D435_basic_config_ D435_basic_config;
//   ROI_Factory Roi_factory;
};

#endif  // COTEK_VISUAL_INCLUDE_COTEK_VISUAL_COTEK_VISUAL_OPTIONS_H_