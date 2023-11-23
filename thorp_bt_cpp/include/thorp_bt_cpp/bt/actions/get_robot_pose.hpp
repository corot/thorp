#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"

#include <mbf_msgs/ExePathAction.h>

#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class GetRobotPose : public BT::StatefulActionNode
{
public:
  GetRobotPose(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& pnh)
    : StatefulActionNode(name, config), timeout_(), pnh_(pnh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("timeout"), BT::OutputPort<unsigned int>("error"),
             BT::OutputPort<geometry_msgs::PoseStamped>("robot_pose") };
  }

  BT::NodeStatus onStart() override
  {
    timeout_.fromSec(utils::getInput<double>(*this, pnh_, "timeout"));
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.frame_id = "base_footprint";
    robot_pose.pose.orientation.w = 1.0;
    if (ttk::TF2::instance().transformPose("map", robot_pose, robot_pose, timeout_))
    {
      setOutput("robot_pose", robot_pose);
      return BT::NodeStatus::SUCCESS;
    }

    ROS_ERROR_NAMED(name(), "Could not get the current robot pose");
    utils::setError(*this, mbf_msgs::ExePathResult::TF_ERROR);
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
  }

private:
  ros::Duration timeout_;
  ros::NodeHandle pnh_;
};
}  // namespace thorp::bt::actions
