#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"

#include <mbf_msgs/ExePathResult.h>
#include <mbf_utility/robot_information.h>

namespace thorp::bt::actions
{
class GetRobotPose : public BT::StatefulActionNode
{
public:
  GetRobotPose(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& pnh,
               const mbf_utility::RobotInformation& robot_info)
    : StatefulActionNode(name, config), robot_info_(robot_info), start_time_(), timeout_(), pnh_(pnh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("timeout"), BT::OutputPort<unsigned int>("error"),
             BT::OutputPort<geometry_msgs::PoseStamped>("robot_pose") };
  }

  BT::NodeStatus onStart() override
  {
    timeout_ = utils::getInput<double>(*this, pnh_, "timeout");
    start_time_ = ros::Time::now();
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    geometry_msgs::PoseStamped robot_pose;
    if (robot_info_.getRobotPose(robot_pose))
    {
      setOutput("robot_pose", robot_pose);
      return BT::NodeStatus::SUCCESS;
    }

    const auto dt = (ros::Time::now() - start_time_).toSec();
    if (dt > timeout_)
    {
      ROS_ERROR_NAMED(name(), "Could not get the current robot pose after %.1f seconds", dt);
      utils::setError(*this, mbf_msgs::ExePathResult::TF_ERROR);
      return BT::NodeStatus::FAILURE;
    }

    ROS_WARN_THROTTLE_NAMED(1.0, name(), "Waiting for TF for %.1f seconds... (transform_tolerance %f)", dt, timeout_);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
  }

private:
  const mbf_utility::RobotInformation& robot_info_;
  ros::Time start_time_;
  double timeout_;
  ros::NodeHandle pnh_;
};
}  // namespace thorp::bt::actions
