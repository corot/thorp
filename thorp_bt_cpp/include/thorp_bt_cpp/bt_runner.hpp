#pragma once

#include <ros/ros.h>

#include <behaviortree_cpp_v3/bt_factory.h>

// bt publisher
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

namespace thorp::bt
{
class Runner
{
public:
  Runner();

  void run();

  /** Factory to create the BT. It's static, so the custom nodes can register themselves with static initialization */
  static BT::BehaviorTreeFactory bt_factory_;

private:
  ros::NodeHandle pnh_;

  /** Running app name. */
  std::string app_name_;

  /** Behavior tree. */
  std::optional<BT::Tree> bt_;

  /** Publisher to attach Groot and visualize the bt in real time. */
  std::optional<BT::PublisherZMQ> bt_pub_zmq_;

  /** Publisher to save the bt to a file and visualize with Groot at a later time. */
  std::optional<BT::FileLogger> bt_pub_file_;

  /** Tick rate of the bt. */
  double tick_rate_;
};

}  // namespace thorp::bt
