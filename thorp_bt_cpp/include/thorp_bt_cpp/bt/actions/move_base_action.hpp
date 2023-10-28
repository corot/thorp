#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"
#include "thorp_bt_cpp/bt/actions/action_client_node.hpp"

#include <mbf_msgs/MoveBaseAction.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;

namespace thorp::bt::actions
{
class MoveBaseAction : public BT::SimpleActionClientNode<mbf_msgs::MoveBaseAction>
{
public:
  MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config) : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<unsigned int>("error"),
             BT::OutputPort<std::optional<mbf_msgs::MoveBaseFeedback>>("feedback") };
  }

  virtual void onFeedback(const mbf_msgs::MoveBaseFeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<mbf_msgs::MoveBaseFeedback>(*feedback));
  }

  virtual BT::NodeStatus onAborted(const mbf_msgs::MoveBaseResultConstPtr& res) override
  {
    ROS_WARN_NAMED(name(), "MoveBase failed at %.2f, %.2f, %.2f; distance to goal: %.2f, angle to goal: %.2f",
                   res->final_pose.pose.position.x, res->final_pose.pose.position.y,
                   ttk::yaw(res->final_pose), res->dist_to_goal, res->angle_to_goal);

    ROS_WARN_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());

    utils::setError(*this, res->outcome);

    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace thorp::bt::actions
