#pragma once

#include <cstdint>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

namespace thorp::bt
{
/**
 * Base for the nodes calling Nav2's actions, with Nav2's BT nodes' outputs: the action's error code on error_code_id,
 * NONE if it succeeds or is canceled, and GOAL_REJECTED if the server refuses the goal.
 */
template <class ActionT>
class Nav2ActionNode : public BT::RosActionNode<ActionT>
{
public:
  using Result = typename ActionT::Result;

  Nav2ActionNode(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : BT::RosActionNode<ActionT>(name, action_name, config)
  {
  }

  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    addition.insert(BT::OutputPort<uint16_t>("error_code_id", "the action's error code"));
    return BT::RosActionNode<ActionT>::providedBasicPorts(addition);
  }

protected:
  BT::NodeStatus onSucceeded(const Result& /*result*/) override
  {
    this->setOutput("error_code_id", Result::NONE);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onAborted(const Result& result) override
  {
    RCLCPP_ERROR(logger(*this), "%s failed with error %d: %s", this->action_name_.c_str(), result.error_code,
                 result.error_msg.c_str());
    this->setOutput("error_code_id", result.error_code);
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onCanceled() override
  {
    this->setOutput("error_code_id", Result::NONE);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRejected() override
  {
    this->setOutput("error_code_id", Result::GOAL_REJECTED);
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace thorp::bt
