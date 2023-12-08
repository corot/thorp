#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/actions/action_client_node.hpp"

#include <mbf_msgs/RecoveryAction.h>

namespace thorp::bt::actions
{

/**
 * Calls MBF's recovery action with the given goal.
 *
 * @param[in]  action_name    Name of the ROS action
 * @param[in]  server_timeout Timeout (sec) to connect to the server
 * @param[in]  goal           Goal to send to the recovery action
 * @param[out] error          Outcome of recovery action if failure
 * @param[out] feedback       Feedback provided by recovery action
 *
 * @return  SUCCESS if action succeeded
 *          FAILURE otherwise
 */
class RecoveryAction : public BT::SimpleActionClientNode<mbf_msgs::RecoveryAction>
{
public:
  RecoveryAction(const std::string& name, const BT::NodeConfiguration& config) : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<unsigned int>("error"),
             BT::OutputPort<std::optional<mbf_msgs::RecoveryFeedback>>("feedback") };
  }

  virtual void onFeedback(const mbf_msgs::RecoveryFeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<mbf_msgs::RecoveryFeedback>(*feedback));
  }

  virtual BT::NodeStatus onAborted(const mbf_msgs::RecoveryResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace thorp::bt::actions
