#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <mbf_msgs/ExePathAction.h>

#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class Sleep : public BT::StatefulActionNode
{
public:
  Sleep(const std::string& name, const BT::NodeConfiguration& config)
    : StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("duration") };
  }

  BT::NodeStatus onStart() override
  {
    start_time_ = ros::Time::now();
    duration_.fromSec(*getInput<double>("duration"));
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    return ros::Time::now() - start_time_ < duration_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
  }

private:
  ros::Time start_time_;
  ros::Duration duration_;
};
}  // namespace thorp::bt::actions
