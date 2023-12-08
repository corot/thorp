#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/actions/action_client_node.hpp"

#include <mbf_msgs/ExePathAction.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{

/**
 * Calls MBF's exe_path action with the given goal.
 *
 * @param[in]  action_name    Name of the ROS action
 * @param[in]  server_timeout Timeout (sec) to connect to the server
 * @param[in]  goal           Goal to send to the exe_path action
 * @param[out] error          Outcome of exe_path action if failure
 * @param[out] feedback       Feedback provided by exe_path action
 *
 * @return  SUCCESS if action succeeded
 *          FAILURE otherwise
 */
class ExePathAction : public BT::SimpleActionClientNode<mbf_msgs::ExePathAction>
{
public:
  ExePathAction(const std::string& name, const BT::NodeConfiguration& config) : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("controller"), BT::InputPort<nav_msgs::Path>("path"),
             BT::OutputPort<unsigned int>("error"),
             BT::OutputPort<std::optional<mbf_msgs::ExePathFeedback>>("feedback") };
  }

  bool setGoal(GoalType& goal) override
  {
    goal.controller = *getInput<std::string>("controller");
    goal.path = *getInput<nav_msgs::Path>("path");
    return true;
  }

  void onFeedback(const mbf_msgs::ExePathFeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<mbf_msgs::ExePathFeedback>(*feedback));
  }

  BT::NodeStatus onAborted(const mbf_msgs::ExePathResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "ExePath failed at %.2f, %.2f, %.2f; distance to goal: %.2f, angle to goal: %.2f",
                    res->final_pose.pose.position.x, res->final_pose.pose.position.y, ttk::yaw(res->final_pose),
                    res->dist_to_goal, res->angle_to_goal);

    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace thorp::bt::actions
