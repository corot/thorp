#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/actions/action_client_node.hpp"

#include <thorp_msgs/FollowPoseAction.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class FollowPose : public BT::SimpleActionClientNode<thorp_msgs::FollowPoseAction>
{
public:
  FollowPose(const std::string& name, const BT::NodeConfiguration& config) : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<float>("time_limit"),
             BT::InputPort<float>("distance"),
             BT::InputPort<bool>("stop_at_distance") };
  }

  bool setGoal(GoalType& goal) override
  {
    goal.time_limit.fromSec(*getInput<float>("time_limit"));
    goal.distance = *getInput<float>("distance");
    goal.stop_at_distance = *getInput<bool>("stop_at_distance");
    return true;
  }

  void onFeedback(const thorp_msgs::FollowPoseFeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<thorp_msgs::FollowPoseFeedback>(*feedback));
  }

  BT::NodeStatus onAborted(const thorp_msgs::FollowPoseResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "FollowPose failed with error %d", res->outcome);

// TODO add message field    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace thorp::bt::actions
