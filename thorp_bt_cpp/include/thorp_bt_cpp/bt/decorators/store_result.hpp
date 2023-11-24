#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/decorator_node.h>

namespace thorp::bt::decorators
{
/** Stores the result of the child as a bool in the blackboard. */
class StoreResult : public BT::DecoratorNode
{
public:
  StoreResult(const std::string& name, const BT::NodeConfiguration& config)
    : DecoratorNode(name, config)
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
};
}  // namespace thorp::bt::decorators
