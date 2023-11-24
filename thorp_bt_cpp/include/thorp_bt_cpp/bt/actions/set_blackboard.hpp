#pragma once

#include <behaviortree_cpp_v3/action_node.h>

namespace thorp::bt::actions
{
/** Adapted from BT::SetBlackboard to work with any type. */
template <typename T>
class SetBlackboard : public BT::SyncActionNode
{
public:
  SetBlackboard(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& pnh)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<T>("input"), BT::OutputPort<T>("output") };
  }

private:
  virtual BT::NodeStatus tick() override
  {
    const auto val = getInput<T>("input");
    setOutput("output", val);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace thorp::bt::actions
