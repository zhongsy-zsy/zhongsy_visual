/**
 * Copyright (c) 2020 COTEK Inc. All rights reserved.
 */

#ifndef COTEK_VISUAL_INCLUDE_NODE_COTEK_VISUAL_NODE_H
#define COTEK_VISUAL_INCLUDE_NODE_COTEK_VISUAL_NODE_H

#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include "cotek_visual/d435_driver.h"
#include "cotek_visual/dynamicConfig.h"

RealSenseOption ConfigFileToVisualOption(std::string config_string);

class VisualNode final {
 public:
  VisualNode() = delete;
  VisualNode(std::shared_ptr<cotek_visual::D435> d435_driver) {
    d435_driver_ptr_ = d435_driver;

    // 定时事件
    // TODO(zhongsy) 设置错误信息发送给diagnostic
  }
  ~VisualNode() {}
  //   void PublishDistData(const ros::TimerEvent& e) { PublishData(); }

  inline bool Init(ros::NodeHandle* nh) {
    // subscribers_.emplace_back(nh->subscribe<cotek_msgs::safety_setting>(
    //     cotek_topic::kSafetySettingTopic, kTopicReciveCacheSize,
    //     boost::bind(&VisualNode::HandleSafetySettingMessage, this, _1)));
    // update_visual_config_service_server_ =
    //     nh->advertiseService(cotek_config::kUpdateVisualConfigService,
    //                          &VisualNode::UpdateVisualConfig, this);

    return d435_driver_ptr_->Init();
  }

  void dynamic_callback(cotek_dynamic::dynamicConfig& config) {
    d435_driver_ptr_->dynamic_callback(config);
  }

  inline void Run() { d435_driver_ptr_->Run(); }

  //   bool UpdateVisualConfig(cotek_msgs::update_realsense_config::Request&
  //   req,
  //                           cotek_msgs::update_realsense_config::Response&
  //                           res) {
  //     // 更新视觉配置
  //     // std::string navigation_config_data =
  //     //     BasicConfigHelper::Instance().GetConfig(
  //     //         cotek_config::ConfigType::VISUAL_CONFIG);
  //     // ROS_INFO("update navigation option.");
  //     // return d435_driver_ptr_->UpdateOption(
  //     //     ConfigFileToVisualOption(navigation_config_data));
  //   }

 private:
  ros::NodeHandle nh;

  std::shared_ptr<cotek_visual::D435> d435_driver_ptr_;
  std::vector<ros::Subscriber> subscribers_;
  ros::ServiceServer update_visual_config_service_server_;
  ros::Timer pub_dist_timer_;
  ros::Timer node_diagnostic_timer_;
};

RealSenseOption Load_visaul_config() {
  ros::NodeHandle nh;

  RealSenseOption option = {};
  option.D435_basic_config.arethread.resize(2);
  option.D435_basic_config.up_to_nums.resize(2);
  nh.getParam("arethread_0", option.D435_basic_config.arethread[0]);
  nh.getParam("arethread_1", option.D435_basic_config.arethread[1]);
  nh.getParam("enable_visual_debug",
              option.D435_basic_config.enable_visual_debug);
  nh.getParam("left_delta_of_D435",
              option.D435_basic_config.left_delta_of_D435);
  nh.getParam("nums", option.D435_basic_config.nums);
  nh.getParam("threshold_x", option.D435_basic_config.threshold_x);
  nh.getParam("threshold_y", option.D435_basic_config.threshold_y);
  nh.getParam("up_num", option.D435_basic_config.up_num);
  nh.getParam("up_pile", option.D435_basic_config.up_pile);
  nh.getParam("up_to_nums_0", option.D435_basic_config.up_to_nums[0]);
  nh.getParam("up_to_nums_1", option.D435_basic_config.up_to_nums[1]);
  nh.getParam("enable_avoid", option.D435_basic_config.enable_avoid);

  ROS_INFO("arethread_0 : %d", option.D435_basic_config.arethread[0]);
  ROS_INFO("arethread_1 : %d", option.D435_basic_config.arethread[1]);
  ROS_INFO("enable_visual_debug : %d",
           option.D435_basic_config.enable_visual_debug);
  ROS_INFO("left_delta_of_D435 : %f",
           option.D435_basic_config.left_delta_of_D435);
  ROS_INFO("nums : %d ", option.D435_basic_config.nums);
  ROS_INFO("threshold_x : %d", option.D435_basic_config.threshold_x);
  ROS_INFO("threshold_y : %d", option.D435_basic_config.threshold_y);
  ROS_INFO("up_num  : %d", option.D435_basic_config.up_num);
  ROS_INFO("up_pile : %f", option.D435_basic_config.up_pile);
  ROS_INFO("up_to_nums_0 : %f", option.D435_basic_config.up_to_nums[0]);
  ROS_INFO("up_to_nums_1 : %f", option.D435_basic_config.up_to_nums[1]);
  ROS_INFO("enable_avoid : %d", option.D435_basic_config.enable_avoid);

  return option;
}
#endif  // COTEK_VISUAL_INCLUDE_NODE_COTEK_VISUAL_NODE_H
