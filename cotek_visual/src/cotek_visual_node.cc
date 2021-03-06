#include "node/cotek_visual_node.h"

#include "cotek_visual/dynamicConfig.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cotek_visual");
  // Set ros log level:
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::NodeHandle nh;

  // TODO(zhongsy) :把配置和更新配置放到这个文件
  //   添加设置避障地图函数

  RealSenseOption option;

  option = Load_visaul_config();

  auto tmp = std::make_shared<cotek_visual::D435>(option);

  VisualNode node(tmp);
  dynamic_reconfigure::Server<cotek_dynamic::dynamicConfig> server;
  dynamic_reconfigure::Server<cotek_dynamic::dynamicConfig>::CallbackType
      callback;
  callback = boost::bind(&VisualNode::dynamic_callback, &node, _1);
  server.setCallback(callback);

  if (!node.Init(&nh)) {
    ROS_ERROR("cotek_visual");
  }
  node.Run();
  ros::spin();
  return 0;
}
