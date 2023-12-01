#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

namespace thorp::bt::conditions
{
/**
 * Checks if the given list is empty.
 *
 * @param[in] list List to evaluate
 *
 * @return  SUCCESS if the given list doesn't exist or it's empty
 *          FAILURE if the given list is not empty
 */
template <typename T>
class IsListEmpty : public BT::ConditionNode
{
public:
  IsListEmpty(const std::string& name, const BT::NodeConfiguration& config)
    : ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<T>>("list") };
  }

  BT::NodeStatus tick() override
  {
    auto list = getInput<std::vector<T>>("list");
    return !list || list->empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
}  // namespace thorp::bt::conditions
