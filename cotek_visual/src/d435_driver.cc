
#include "d435_driver.h"

#include <algorithm>
#include <utility>

namespace cotek_visual {

namespace {
constexpr float pi = 3.1415926;
constexpr size_t kThreadSize = 2;
constexpr int kLeftEdge = 200;
constexpr int kRightEdge = 510;
constexpr int kTopEdge = 0;
constexpr int kBelowEdge = 480;
constexpr int kFlagThreads = 0;
constexpr int kIterTimes = 3;        // 迭代次数
constexpr int kSamplesNumsUp = 100;  // 计算阈值样本增量
constexpr int kWidth = 848;
constexpr int kHeight = 480;
constexpr int kFps = 90;  // 帧数最大能支持90

}  // namespace

D435::D435()
    : RSAvoidMAP_(RSAvoidMapType::NONE),
      thread_pool_(kThreadSize),
      min_distance_(4000),
      enable_avoid_(false),
      thread_sleep_frequency_(20) {}

D435::D435(RealSenseOption option)
    : RSAvoidMAP_(RSAvoidMapType::NONE),
      thread_pool_(kThreadSize),
      min_distance_(4000),
      enable_avoid_(false),
      thread_sleep_frequency_(20) {
  Realsense_options_ = option;
  ros::NodeHandle nh;
  //   mindistance_pub_ = nh.advertise<cotek_msgs::d435_feedback>(
  //       cotek_topic::kVisualProtectTopic, 2);
}

D435::~D435() {
  if (run_executor_) {
    run_executor_->join();
    run_executor_ = nullptr;
  }
}

bool D435::Init() {
  try {
    image_transport_ptr_ =
        std::make_shared<image_transport::ImageTransport>(n_);

    rs2::context ctx;
    // 获取设备列表
    rs2::device_list dev_list;
    dev_list = ctx.query_devices();
    if (dev_list.size() == 0) {
      ROS_ERROR("D435 not detected");
      return false;
    }
    pipe_ = rs2::pipeline(ctx);

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, kWidth, kHeight, RS2_FORMAT_Z16, kFps);
    pipe_.start(cfg);

    // filters 初始化
    if (decimation_filter_.supports(rs2_option::RS2_OPTION_FILTER_MAGNITUDE)) {
      rs2::option_range option_range = decimation_filter_.get_option_range(
          rs2_option::RS2_OPTION_FILTER_MAGNITUDE);
      decimation_filter_.set_option(
          rs2_option::RS2_OPTION_FILTER_MAGNITUDE,
          option_range.min);  // 1(min) is not downsampling
    }

    // Set Spatial Filter Option
    if (spatial_filter_.supports(rs2_option::RS2_OPTION_HOLES_FILL)) {
      rs2::option_range option_range =
          spatial_filter_.get_option_range(rs2_option::RS2_OPTION_HOLES_FILL);
      spatial_filter_.set_option(rs2_option::RS2_OPTION_HOLES_FILL,
                                 option_range.max);  // 5(max) is fill all holes
    }

    image_pub_ = image_transport_ptr_->advertise("/realsense/convex", 1);
    image_pub2_ = image_transport_ptr_->advertise("/realsense/rectangle", 1);

    are_threshold_ = Realsense_options_.D435_basic_config.arethread;

    ROI_UP_ = cv::Rect(200, 0, 200, 240);
    ROI_DOWN_ = cv::Rect(100, 240, 500, 240);

    frames_ = pipe_.wait_for_frames();

    dprofile_ = frames_.get_depth_frame().get_profile();

    depth_intrin_ =
        rs2::video_stream_profile(dprofile_).get_intrinsics();  // 获取内参

    return true;
  } catch (...) {
    ROS_ERROR("WRONG !!!");
    return false;
  }
}

// void D435::ChangeSleepFrequency(
//     const cotek_msgs::move_feedback::ConstPtr &feedback) {
//   float velocity = feedback->actual_velocity;
//   if (velocity < 0.3) {
//     thread_sleep_frequency_ = 50;
//   } else if (velocity < 0.5) {
//     thread_sleep_frequency_ = 30;
//   } else if (velocity < 1.0) {
//     thread_sleep_frequency_ = 20;
//   } else {
//     thread_sleep_frequency_ = 20;
//   }
// }

// void D435::SetAvoidMap(const cotek_msgs::safety_setting::ConstPtr &setting) {
//   // uint32_t data = static_cast<uint32_t>(setting->avoid_map);
//   switch (static_cast<AvoidMapType>(setting->avoid_map)) {
//     case AvoidMapType::FORWARD:
//       ROI_UP_ = Realsense_options_.Roi_factory.ROI_STRAIGHT_UP_;
//       ROI_DOWN_ = Realsense_options_.Roi_factory.ROI_STRAIGHT_DOWN_;
//       enable_avoid_ = true;
//       break;
//     case AvoidMapType::FORWARD_LEFT:
//     case AvoidMapType::BACK_LEFT:
//       ROI_UP_ = Realsense_options_.Roi_factory.ROI_LEFT_UP_;
//       ROI_DOWN_ = Realsense_options_.Roi_factory.ROI_LEFT_DOWN_;
//       enable_avoid_ = true;
//       break;
//     case AvoidMapType::FORWARD_RIGHT:
//     case AvoidMapType::BACK_RIGHT:
//       ROI_UP_ = Realsense_options_.Roi_factory.ROI_RIGHT_UP_;
//       ROI_DOWN_ = Realsense_options_.Roi_factory.ROI_RIGHT_DOWN_;
//       enable_avoid_ = true;
//       break;
//     case AvoidMapType::AVOID_DIY_MAP_25:
//       ROI_UP_ = cv::Rect(300, 200, 100, 50);
//       ROI_DOWN_ = cv::Rect(300, 250, 100, 50);
//       enable_avoid_ = true;
//       break;
//     case AvoidMapType::NONE:
//     case AvoidMapType::INIT:
//       ROI_UP_ = Realsense_options_.Roi_factory.ROI_STRAIGHT_UP_;
//       ROI_DOWN_ = Realsense_options_.Roi_factory.ROI_STRAIGHT_DOWN_;
//       min_distance_ = 4000;
//       enable_avoid_ = false;
//       break;
//     default:
//       ROI_UP_ = Realsense_options_.Roi_factory.ROI_STRAIGHT_UP_;
//       ROI_DOWN_ = Realsense_options_.Roi_factory.ROI_STRAIGHT_DOWN_;
//       min_distance_ = 4000;
//       enable_avoid_ = false;
//       break;
//   }
// }

void D435::Run() {
  run_executor_ =
      std::make_shared<std::thread>(std::bind(&D435::HandleFeedbackData, this));
}

/* 更新参数 */
// bool D435::update_realsense_configs(
//     cotek_msgs::update_realsense_config::Request &req,
//     cotek_msgs::update_realsense_config::Response &res) {
//   return UpdateConfig();
// }

bool D435::judge_file(const std::string &name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

/* 获取阈值 */
std::vector<double> D435::calculate_max_threshold(
    cv::Mat mean_depth, const std::vector<cv::Mat> &raw_data) {
  std::vector<double> threshold;  // 用来存放阈值
  for (int i = 0; i < mean_depth.rows; i++) {
    double max_diff = 0;
    for (int j = 0; j < mean_depth.cols; j++) {
      /* 求出最大的偏差值 */
      for (int k = 0; k < raw_data.size(); k++) {
        double diff_tmp = static_cast<double>(mean_depth.at<ushort>(i, j)) -
                          static_cast<double>(raw_data[k].at<ushort>(i, j));
        if (diff_tmp < 0) {
          continue;
          //   std::max(max_diff, std::fabs(diff_tmp));
        } else {
          max_diff = std::max(max_diff, diff_tmp);
        }
      }
    }
    threshold.push_back(max_diff);
    max_diff = 0;
  }

  return threshold;
}

/*  获取障碍物的最近距离  */
void D435::GetData(void *data) {
  auto cdata = static_cast<double *>(data);
  *cdata = min_distance_;
}

/* 获取848x640的深度图像 */
void D435::get_depth() {
  frames_ = pipe_.wait_for_frames();
  rs2::depth_frame depth_frame = frames_.get_depth_frame();
  rs2::frame filtered_frame = depth_frame;
  // 应用抽取滤波器（下采样）

  filtered_frame = decimation_filter_.process(filtered_frame);
  //   // 从深度帧转换视差帧
  rs2::disparity_transform disparity_transform(true);
  filtered_frame = disparity_transform.process(filtered_frame);

  // 应用空间滤镜（保留边缘的平滑，补孔）
  filtered_frame = spatial_filter_.process(filtered_frame);

  // 应用时间过滤器（使用多个先前的帧进行平滑处理）
  filtered_frame = temporal_filter_.process(filtered_frame);

  // 从视差帧变换深度帧
  rs2::disparity_transform depth_transform(false);
  filtered_frame = depth_transform.process(filtered_frame);

  cv::Mat depth(cv::Size(kWidth, kHeight), CV_16UC1,
                (void *)filtered_frame.get_data(), cv::Mat::AUTO_STEP);

  depth.copyTo(depth_data_);
}

/*  发现凸包 */
Contour D435::FindObstacle(const std::vector<cv::Mat> &depth, int thresh,
                           int max_thresh, const std::vector<int> &areas) {
  Contour contours;
  Contour contours_result;
  std::vector<cv::Vec4i> hierarchy;

  // 阈值分割
  for (int i = 0; i < depth.size(); i++) {
    cv::threshold(depth[i], depth[i], thresh, 255, cv::THRESH_BINARY_INV);
  }

  // 寻找轮廓
  // 绘出轮廓及其凸包
  for (int h = 0; h < depth.size(); h++) {
    findContours(depth[h], contours, hierarchy,
                 CV_RETR_TREE,  // 找到第i副图像的轮廓
                 CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    /// 对每个轮廓计算其凸包
    std::vector<std::vector<cv::Point>> hull(contours.size());
    for (uint i = 0; i < contours.size(); i++) {
      convexHull(cv::Mat(contours[i]), hull[i], false);
    }

    for (int i = 0; i < contours.size(); i++) {
      if (contourArea(contours[i]) < areas[h] ||
          contourArea(contours[i]) >
              306000)  // 面积大于或小于area的凸包，可忽略
        continue;
      contours_result.push_back(hull[i]);
    }

    if (Realsense_options_.D435_basic_config.enable_visual_debug) {
      cv::Mat drawing =
          cv::Mat::zeros(kHeight, kWidth, CV_8UC3);  // 用于画图显示
      cv::RNG rng(12345);
      for (int i = 0; i < contours.size(); i++) {
        if (contourArea(contours[i]) < areas[h] ||
            contourArea(contours[i]) >
                306000)  // 面积大于或小于area的凸包，可忽略
          continue;
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                                      rng.uniform(0, 255));
        drawContours(drawing, contours, i, color, 1, 8,
                     std::vector<cv::Vec4i>(), 0, cv::Point());
        drawContours(drawing, hull, i, color, 1, 8, std::vector<cv::Vec4i>(), 0,
                     cv::Point());
      }
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
      image_pub_.publish(msg);
      contours.clear();
    }
  }
  return contours_result;
}

/* 计算最近距离 */
void D435::CalculateMinDist(const Contour &contour, const cv::Mat &origin_frame,
                            float threshold_x, float threshold_y) {
  // TODO(zhongsy):
  // X的距离是到左眼的距离，所以左右距离是不一样的，还需要进行改进
  // cv::Mat drawing = cv::Mat::zeros(cv::Size(kWidth, kHeight), CV_8UC3);
  std::vector<cv::Rect> ve_rect;

  // change min_distance_ sholud  get mutex
  std::unique_lock<std::mutex> lock(mutex_);

  if (contour.empty()) {
    min_distance_ = KSaftyDist;
  } else {
    for (int i = 0; i < contour.size(); i++) {
      cv::Rect rect;
      rect = cv::boundingRect(contour[i]);
      //   if (rect.area() > 304000) {continue;}
      ve_rect.emplace_back(rect);
    }
    // 去除重复框
    std::sort(
        ve_rect.begin(), ve_rect.end(),
        [](cv::Rect a, cv::Rect b) -> bool { return a.area() > b.area(); });
    for (auto i = ve_rect.begin(); i < ve_rect.end(); i++) {
      for (auto j = i + 1; j < ve_rect.end(); j++) {
        if (isInside((*i), (*j))) {
          ve_rect.erase(j);
          j--;
        }
      }
    }
/* 进行rect减半操作 */
#if 0
    for (auto i = ve_rect.begin(); i < ve_rect.end(); i++) {
      (*i) = (*i) + cv::Point((*i).width / 2, (*i).height / 2);
      (*i) = (*i) + cv::Size(-(*i).width / 2, -(*i).height / 2);
    }
#endif

    // 计算每一个rect中的最小距离
    std::vector<double> res;
    if (!ve_rect.empty()) {
      for (int i = 0; i < ve_rect.size(); i++) {
        cv::Mat vect_roi = origin_frame(ve_rect[i]);
        int x_delta = ve_rect[i].x;
        int y_delta = ve_rect[i].y;

        // cv::rectangle(drawing, ve_rect[i], cv::Scalar(0, 0, 255));
        for (int i = 0; i < vect_roi.rows; i++) {
          for (int j = 0; j < vect_roi.cols; j++) {
            // 过滤零点
            ushort depth_dis = vect_roi.at<ushort>(i, j);
            if (depth_dis == 0 || depth_dis > KMaxDist) {
              continue;
            }
            float Z = static_cast<float>(vect_roi.at<ushort>(i, j));
            float X =
                (static_cast<float>((j + x_delta) * Z) -
                 depth_intrin_.ppx * Z) /
                depth_intrin_
                    .fx;  // 这是考虑的没有畸变的情况，如果有畸变那么就是另外的情况了

            if (std::fabs(X + Realsense_options_.D435_basic_config
                                  .left_delta_of_D435) > threshold_x) {
              depth_dis = KMaxDist;
              continue;
            }

            float Y = (static_cast<float>((i + y_delta) * Z) -
                       depth_intrin_.ppy * Z) /
                      depth_intrin_.fy;

            // if (i == vect_roi.rows / 2 && j == vect_roi.cols / 2) {
            //   ROS_ERROR_THROTTLE(
            //       1, "X is %f %f %f",
            //       X +
            //       Realsense_options_.D435_basic_config.left_delta_of_D435, X,
            //       Realsense_options_.D435_basic_config.left_delta_of_D435);
            // }
            // 进行相机坐标到车体坐标的转换
            Z = -std::sin(install_ration_angle_) * Y +
                std::cos(install_ration_angle_) * Z;

#if 0  // 目前不需要Y(车体上下方向)的值
            Y = std::cos(install_ration_angle_) * Y +
                std::sin(install_ration_angle_) * Z;
#endif

            vect_roi.at<ushort>(i, j) = static_cast<ushort>(Z);
          }
        }

        // sensor_msgs::ImagePtr msg =
        //     cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing)
        //         .toImageMsg();
        // image_pub2_.publish(msg);
        double min_dis;
        cv::Point min_point;
        std::vector<int> tmp;

        // for(int i=0;i<3;i++)
        // TODO(zhongsy)  考虑处理速度优化，目前仅考虑最近的一个点
        while (tmp.size() <= 1) {
          cv::minMaxLoc(vect_roi, &min_dis, NULL, &min_point, NULL);
          //  std::cout<<"min_dis_roi"<<min_dis<<std::endl;
          // 优化获取的是前面3个最小点的平均值
          if (tmp.empty()) {
            tmp.emplace_back(min_dis);
            vect_roi.at<ushort>(min_point) = KSaftyDist;
          } else {
            if (std::fabs(min_dis - tmp.back()) < 40) {
              tmp.emplace_back(min_dis);
              vect_roi.at<ushort>(min_point) = KSaftyDist;
            } else {
              // 是噪声点
              tmp.pop_back();
            }
          }
          // std::cout<<tmp.size()<<std::endl;
        }

        min_dis = 0;
        for (auto average : tmp) {
          min_dis += average;
        }
        res.emplace_back(min_dis / 2.0);
      }

      if (!res.empty()) {
        min_distance_ = *std::min_element(res.begin(), res.end());
      } else {
        min_distance_ = KSaftyDist;
      }
    } else {
      min_distance_ = KSaftyDist;
    }
    // cotek_msgs::d435_feedback msg;
    // msg.mindistance = min_distance_;
    // mindistance_pub_.publish(msg);
  }
}

/* 像素坐标转到相机坐标 */
cv::Vec3f D435::pixel_to_world(cv::Vec3f point) {
  cv::Vec3f result;
  result[0] =
      (point[0] * point[2] - depth_intrin_.ppx * point[2]) / depth_intrin_.fx;
  result[1] =
      (point[1] * point[2] - depth_intrin_.ppy * point[2]) / depth_intrin_.fy;
  result[2] = point[2];

  result[2] = -std::sin(install_ration_angle_) * result[1] +
              std::cos(install_ration_angle_) * result[2];
  result[1] = std::cos(install_ration_angle_) * result[1] +
              std::sin(install_ration_angle_) * result[2];

  return result;
}

void D435::Loadvisualconfig() {
  if (judge_file("/home/cotek/angle_data.csv")) {
    ROS_INFO("load visual_angle");
    std::ifstream inFile("/home/cotek/angle_data.csv", std::ios::in);
    std::string value;
    getline(inFile, value);  // 读取整行进value中 读取第一行
    std::stringstream ss(value);
    std::string str;
    while (getline(ss, str, ',')) {  // 以逗号为分隔读取string
      install_ration_angle_ =
          static_cast<float>(atof(str.c_str()));  // string转为double
      install_ration_angle_ = install_ration_angle_ / 180.0 * pi;
    }
    ROS_INFO("load visual_angle finished");

  } else {
    // node_status_manager_ptr_->SetNodeStatus(VisualNodeStatus::LOCK_OF_ANGLE);
    // ROS_ERROR("Please calibrate visual_angle");
  }

  if (judge_file("/home/cotek/mean_depth.png") &&
      judge_file("/home/cotek/threshold.csv")) {
    ROS_INFO("load visual_background");
    install_mean_depth_average_ =
        cv::imread("/home/cotek/mean_depth.png", cv::IMREAD_ANYDEPTH);
    // for (int i = 0; i < tmp.rows; i++) {
    //   for (int j = 0; j < tmp.cols; j++) {
    //     install_mean_depth_average_.at<ushort>(i, j) = tmp.at<ushort>(i, j);
    //   }
    // }

    // std::cout << "mean_depth" << install_mean_depth_average_ << std::endl;
    std::ifstream inFile("/home/cotek/threshold.csv", std::ios::in);
    std::string value;
    std::vector<double> data;
    getline(inFile, value);  // 读取整行进value中 读取第一行
    std::stringstream ss(value);
    std::string str;

    while (getline(ss, str, ',')) {  // 以逗号为分隔读取string
      data.emplace_back(
          static_cast<double>(std::stoi(str.c_str())));  // string转为double
    }

    threshold_data_ = data;

    for (auto value : data) {
      std::cout << value << " ";
    }

    ROS_INFO("load visual_background finished");

  } else {
    // node_status_manager_ptr_->SetNodeStatus(
    //     VisualNodeStatus::LOCK_OF_BACKGROUND);
    ROS_ERROR("Please calibrate vsual_backgriund");
  }
}

/*  处理深度信息函数 */
void D435::HandleDepth(const cv::Mat &origin_frame,
                       const std::vector<cv::Mat> &data) {
  auto &&contour = FindObstacle(data, 170, 255,
                                Realsense_options_.D435_basic_config.arethread);
  CalculateMinDist(contour, origin_frame,
                   Realsense_options_.D435_basic_config.threshold_x,
                   Realsense_options_.D435_basic_config.threshold_y);
}

/* 进行阈值处理 */
void D435::Thresholding(const std::vector<cv::Mat> &data,
                        const cv::Mat &mean_depth,
                        const std::vector<double> &thread_data, int h, int nums,
                        const cv::Rect &Image_ROI, cv::Mat result) {
  int roi_height = Image_ROI.y + Image_ROI.height;
  int roi_width = Image_ROI.x + Image_ROI.width;

  if (h == 0) {  // 下面的一层才需要三张叠加进行滤波
    for (int i = Image_ROI.y; i < roi_height; i++) {
      for (int j = Image_ROI.x; j < roi_width; j++) {
        int count = 0;
        for (int k = 0; k < data.size(); k++) {
          ushort depth_dist = data[k].at<ushort>(i, j);
          if (depth_dist == 0 || mean_depth.at<ushort>(i, j) < depth_dist ||
              depth_dist > KMaxDist) {
            break;
          }
          if (static_cast<double>(mean_depth.at<ushort>(i, j) -
                                  data[k].at<ushort>(i, j)) >
              thread_data[i] *
                  Realsense_options_.D435_basic_config.up_to_nums[h]) {
            count++;
          }
        }
        if (count > nums) {
          result.at<uchar>(i, j) = 0;
        }
      }
    }
  } else if (h == 1) {
    for (int i = Image_ROI.y; i < Image_ROI.height + Image_ROI.y; i++) {
      for (int j = Image_ROI.x; j < Image_ROI.width + Image_ROI.x; j++) {
        ushort depth_dist = data[1].at<ushort>(i, j);
        if (depth_dist == 0 || mean_depth.at<ushort>(i, j) < depth_dist ||
            depth_dist > KMaxDist) {  // 只会选择中间的那一张图片
          continue;
        }
        if (static_cast<double>(mean_depth.at<ushort>(i, j) -
                                data[1].at<ushort>(i, j)) >
            thread_data[i] *
                    Realsense_options_.D435_basic_config.up_to_nums[h] +
                Realsense_options_.D435_basic_config.up_pile) {
          result.at<uchar>(i, j) = 0;
        }
      }
    }
  }
}

// 预处理 阈值处理+闭操作
std::vector<cv::Mat> D435::PreProcess(const std::vector<cv::Mat> &origin_frames,
                                      const cv::Mat &mean_depth_average,
                                      const std::vector<double> &threshold_data,
                                      int up_num, int nums,
                                      const cv::Rect &ROI_UP,
                                      const cv::Rect &ROI_DOWN) {
  std::vector<cv::Mat> deal_result(2, cv::Mat());

#if 1
  // 先进行腐蚀处理
  for (int h = 0; h < origin_frames.size(); h++) {
    cv::Mat element_dilate = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(5, 5));  // 闭操作核的大小
    cv::dilate(origin_frames[h], origin_frames[h], element_dilate);
  }
#endif

  for (int k = 0; k < up_num; k++) {
    cv::Mat result_data(kHeight, kWidth, CV_8UC1, cv::Scalar(255));
    Thresholding(origin_frames, mean_depth_average, threshold_data, k, nums - 1,
                 ROI_UP, result_data);
    Thresholding(origin_frames, mean_depth_average, threshold_data, k, nums - 1,
                 ROI_DOWN, result_data);
    result_data.copyTo(deal_result[k]);
  }

  cv::Mat element = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(3, 3));  // 闭操作核的大小
  for (int k = 0; k < up_num; k++) {
    cv::morphologyEx(deal_result[k], deal_result[k], cv::MORPH_CLOSE,
                     element);  // 闭操作
  }
  return deal_result;
}

void D435::CalculateFrams(std::vector<cv::Mat> origin_frames) {
  ros::Time calculate_start = ros::Time::now();
  // ROS_WARN("calculate_start: %lf", calculate_start.toSec());
  cv::Rect ROI_TMP_UP = ROI_UP_;
  cv::Rect ROI_TMP_DOWN = ROI_DOWN_;

  auto &&temp = PreProcess(
      origin_frames, install_mean_depth_average_, threshold_data_,
      Realsense_options_.D435_basic_config.up_num,
      Realsense_options_.D435_basic_config.nums, ROI_TMP_UP, ROI_TMP_DOWN);
  // ROS_WARN("preprocess_consume: %lf",
  //          (ros::Time::now() - calculate_start).toSec());
  HandleDepth(origin_frames[Realsense_options_.D435_basic_config.nums - 1],
              temp);
  // ROS_WARN("calculate_consume: %lf",
  //          (ros::Time::now() - calculate_start).toSec());
}

void D435::ProduceFrams() {
  ros::Time produce_start = ros::Time::now();
  // ROS_WARN("produce_start: %lf", produce_start.toSec());
  std::vector<cv::Mat> res(Realsense_options_.D435_basic_config.nums,
                           cv::Mat());
  for (int i = 0; i < Realsense_options_.D435_basic_config.nums; i++) {
    frames_ = pipe_.wait_for_frames();

    cv::Mat depth(cv::Size(kWidth, kHeight), CV_16UC1,
                  (void *)frames_.get_depth_frame().get_data(),
                  cv::Mat::AUTO_STEP);
    depth.copyTo(res[i]);
  }
  thread_pool_.enqueue([=]() { (CalculateFrams(res)); });
  // ROS_WARN("produce_consume: %lf", (ros::Time::now() -
  // produce_start).toSec());
}

/*
    线程函数，处理获取的深度信息
 */
void D435::HandleFeedbackData() {
  Loadvisualconfig();
  while (ros::ok()) {
    if (enable_avoid_) ProduceFrams();
    // 降低cpu占用率 睡眠时间应小于控制周期
    std::this_thread::sleep_for(
        std::chrono::milliseconds(thread_sleep_frequency_));
  }
}
}  // namespace cotek_visual
