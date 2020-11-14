/**
 * Copyright (c) 2020 COTEK Inc. All rights reserved.
 */

#ifndef COTEK_VISUAL_INCLUDE_COTEK_VISUAL_D435_DRIVER_H_
#define COTEK_VISUAL_INCLUDE_COTEK_VISUAL_D435_DRIVER_H_

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sys/stat.h>

#include <fstream>
#include <librealsense2/rs.hpp>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

#include "cotek_visual/dynamicConfig.h"
#include "cotek_visual/thread_pool.h"
#include "cotek_visual_options.h"

namespace cotek_visual {
const double KSaftyDist = 4000;
const ushort KMaxDist = 7500;  // 数据超过7.5m不进行处理

using ThreadPool = cotek_common::ThreadPool;
typedef std::vector<std::vector<cv::Point>> Contour;

enum class RSAvoidMapType : int {
  NONE = 0,
  STRAIGHT = 1,
  LEFT = 2,
  RIGHT = 3,
};

class D435 {
 public:
  D435();
  explicit D435(RealSenseOption option);
  ~D435();

  // 实现AbstractDriver的接口
  bool Init();
  void GetData(void *data);
  void Reset() {}
  void Run();
  inline bool UpdateOption(const RealSenseOption &option) {
    Realsense_options_ = option;
    return true;
  }
  void dynamic_callback(cotek_dynamic::dynamicConfig &config);

 private:
  // 自定义接口


  void Loadvisualconfig();
  void Thresholding(const std::vector<cv::Mat> &data, const cv::Mat &mean_depth,
                    const std::vector<double> &thread_data, int h, int nums,
                    const cv::Rect &Image_ROI, cv::Mat result);
  void ProduceFrams();
  void CalculateFrams(std::vector<cv::Mat> origin_frames);
  std::vector<cv::Mat> PreProcess(const std::vector<cv::Mat> &origin_frames,
                                  const cv::Mat &mean_depth_average,
                                  const std::vector<double> &threshold_data,
                                  int up_num, int nums, const cv::Rect &ROI_UP,
                                  const cv::Rect &ROI_DOWN);
  void HandleFeedbackData();
  void get_depth();
  void HandleDepth(const cv::Mat &origin_frame,
                   const std::vector<cv::Mat> &data);
  Contour FindObstacle(const std::vector<cv::Mat> &depth, int thresh,
                       int max_thresh, const std::vector<int> &areas);
  void CalculateMinDist(const Contour &contour, const cv::Mat &origin_frame,
                        float threshold_x, float threshold_y);
  std::vector<double> calculate_max_threshold(
      cv::Mat mean_depth, const std::vector<cv::Mat> &raw_data);
  cv::Vec3f pixel_to_world(cv::Vec3f point);

  bool isInside(cv::Rect rect1, cv::Rect rect2) {
    cv::Rect tmp = rect1 & rect2;
    return (rect1 == tmp || rect2 == tmp);
  }  // 判断两矩形是否包含
  bool judge_file(const std::string &name);

  bool enable_avoid_;

  ThreadPool thread_pool_;

  std::mutex mutex_;
  std::shared_ptr<std::thread> run_executor_;
  rs2::frameset frames_;
  rs2::pipeline pipe_;
  rs2::spatial_filter spatial_filter_;
  rs2::decimation_filter decimation_filter_;
  rs2::temporal_filter temporal_filter_;
  cv::Mat depth_data_;
  double min_distance_;
  std::vector<double> threshold_data_;
  std::vector<int> are_threshold_;
  //   std::vector<double> up_to_nums;
  cv::Mat install_mean_depth_average_;  // 保存背景图
  float install_ration_angle_;          // 安装角度
  rs2_intrinsics depth_intrin_;         // 存放相机内参
  rs2::stream_profile dprofile_;        // 用来存放深度相机参数
  RealSenseOption Realsense_options_;   // 存放RealSense的参数信息
  cv::Rect ROI_UP_;
  cv::Rect ROI_DOWN_;  // 感兴趣区域
  ros::ServiceServer service_updateconfig_;
  ros::ServiceServer service_calibration_angle_;
  ros::ServiceServer service_calibration_background_;
  RSAvoidMapType RSAvoidMAP_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_ptr_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher image_pub2_;
  ros::NodeHandle n_;
  int thread_sleep_frequency_;
  ros::Publisher mindistance_pub_;
};

}  // namespace cotek_visual

#endif  // COTEK_VISUAL_INCLUDE_COTEK_VISUAL_D435_DRIVER_H_
