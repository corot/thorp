#pragma once

#include <behaviortree_cpp_v3/action_node.h>

namespace thorp::bt::conditions
{
/** Check if a key exists the blackboard. */
template <typename T>
class InBlackboard : public BT::ConditionNode
{
public:
  InBlackboard(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<T>("key") };
  }

private:
  virtual BT::NodeStatus tick() override
  {
    return getInput<T>("key") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
}  // namespace thorp::bt::conditions 
