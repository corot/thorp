#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"

namespace thorp::bt::conditions
{
/**
 * Checks if the given boolean is true.
 *
 * @param[in] input Value to evaluate
 *
 * @return  SUCCESS if given input is true
 *          FAILURE if given input is false
 */
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
