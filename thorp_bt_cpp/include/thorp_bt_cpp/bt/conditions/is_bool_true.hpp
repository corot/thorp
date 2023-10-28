#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"

namespace thorp::bt::conditions
{
class IsBoolTrue : public BT::ConditionNode
{
public:
  IsBoolTrue(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& pnh)
    : ConditionNode(name, config), pnh_(pnh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<bool>("input") };
  }

  BT::NodeStatus tick() override
  {
    const auto input = utils::getInput<bool>(*this, pnh_, "input");
    return input ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ros::NodeHandle pnh_;
};
}  // namespace thorp::bt::conditions
