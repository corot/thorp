#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/actions/action_client_node.hpp"

#include <mbf_msgs/GetPathAction.h>

namespace thorp::bt::actions
{

/**
 * Calls MBF's get_path action with the given goal.
 *
 * @param[in]  action_name    Name of the ROS action
 * @param[in]  server_timeout Timeout (sec) to connect to the server
 * @param[in]  goal           Goal to send to the get_path action
 * @param[out] error          Outcome of get_path action if failure
 * @param[out] feedback       Feedback provided by get_path action
 *
 * @return  SUCCESS if action succeeded
 *          FAILURE otherwise
 */
class GetPathAction : public BT::SimpleActionClientNode<mbf_msgs::GetPathAction>
{
public:
  GetPathAction(const std::string& name, const BT::NodeConfiguration& config) : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<unsigned int>("error"),
             BT::OutputPort<std::optional<mbf_msgs::GetPathFeedback>>("feedback") };
  }

  virtual void onFeedback(const mbf_msgs::GetPathFeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<mbf_msgs::GetPathFeedback>(*feedback));
  }

  virtual BT::NodeStatus onAborted(const mbf_msgs::GetPathResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace thorp::bt::actions
