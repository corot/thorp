#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

namespace thorp::bt::conditions
{
/**
 * Checks if the given list is empty.
 *
 * @param[in] list List to evaluate
 *
 * @return  SUCCESS if the given list is empty
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
    std::vector<T> list;
    getInput<std::vector<T>>("list", list);
    return list.empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
}  // namespace thorp::bt::conditions
