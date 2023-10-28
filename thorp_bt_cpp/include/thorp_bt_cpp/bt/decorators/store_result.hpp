#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/decorator_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"

namespace thorp::bt::decorators
{
/** Stores the result of the child as a bool in the blackboard. */
class StoreResult : public BT::DecoratorNode
{
public:
  StoreResult(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& pnh)
    : DecoratorNode(name, config), pnh_(pnh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<bool>("result") };
  }

  BT::NodeStatus tick() override
  {
    // tick child
    BT::NodeStatus child_status = child_node_->executeTick();

    // if child is still running nothing else to do
    if (child_status == BT::NodeStatus::RUNNING)
    {
      return BT::NodeStatus::RUNNING;
    }

    // child has finished execution
    haltChild();

    setOutput("result", child_status == BT::NodeStatus::SUCCESS ? true : false);

    return child_status;
  }

private:
  ros::NodeHandle pnh_;
};
}  // namespace thorp::bt::decorators
