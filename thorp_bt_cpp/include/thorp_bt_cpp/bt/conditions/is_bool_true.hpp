#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

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
  IsBoolTrue(const std::string& name, const BT::NodeConfiguration& config)
    : ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<bool>("input") };
  }

  BT::NodeStatus tick() override
  {
    const auto input = getInput<bool>("input");
    return input && *input ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
}  // namespace thorp::bt::conditions
