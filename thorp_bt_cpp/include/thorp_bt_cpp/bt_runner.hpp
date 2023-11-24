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

private:
  ros::NodeHandle pnh_;

  /** Behavior tree. */
  std::optional<BT::Tree> bt_;

  /** Factory for the BT. */
  BT::BehaviorTreeFactory bt_factory_;

  /** Publisher to attach Groot and visualize the bt in real time. */
  std::optional<BT::PublisherZMQ> bt_pub_zmq_;

  /** Publisher to save the bt to a file and visualize with Groot at a later time. */
  std::optional<BT::FileLogger> bt_pub_file_;

  /** Tick rate of the bt. */
  double tick_rate_;
};

}  // namespace thorp::bt
