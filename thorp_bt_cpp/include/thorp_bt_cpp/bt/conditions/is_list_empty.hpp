#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"

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
  IsListEmpty(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& pnh)
    : ConditionNode(name, config), pnh_(pnh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<T>>("list") };
  }

  BT::NodeStatus tick() override
  {
    std::vector<T> list;
    utils::getInput<std::vector<T>>(*this, pnh_, "list", list);
    return list.empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ros::NodeHandle pnh_;
};
}  // namespace thorp::bt::conditions
