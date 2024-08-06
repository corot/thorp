#pragma once

#include <ros/ros.h>

#include <behaviortree_cpp/bt_factory.h>

// bt publisher
#include <thorp_bt_cpp/bt_ros_logger.hpp>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

namespace thorp::bt
{
class Runner
{
public:
  Runner();

  bool loadTree();

  void run();

  /** Factory to create the BT. It's static, so the custom nodes can register themselves with static initialization */
  static BT::BehaviorTreeFactory bt_factory_;

private:
  ros::NodeHandle pnh_;

  /** Running app name. */
  std::string app_name_;

  /** Behavior tree. */
  std::optional<BT::Tree> bt_;

  /** Publisher to attach Groot2 and visualize the bt in real time. */
  std::optional<BT::Groot2Publisher> bt_pub_groot_;

  /** Publisher to save the bt to a file and visualize with Groot at a later time. */
  std::optional<BT::FileLogger2> bt_pub_file_;

  /** Publisher to a ROS topic. */
  std::optional<BT::RosLogger> bt_pub_topic_;

  /** Tick rate of the bt. */
  double tick_rate_;
};

}  // namespace thorp::bt
