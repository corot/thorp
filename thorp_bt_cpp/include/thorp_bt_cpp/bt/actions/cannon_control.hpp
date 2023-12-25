#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/actions/service_client_node.hpp"

#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

#include <thorp_msgs/CannonCmd.h>

namespace thorp::bt::actions
{

class AimCannon : public BT::SyncActionNode
{
public:
  AimCannon(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::PoseStamped>("robot_pose"),
             BT::InputPort<geometry_msgs::PoseStamped>("target_pose"),
             BT::OutputPort<float>("angle")};
  }

private:
  BT::NodeStatus tick() override
  {
    geometry_msgs::PoseStamped robot_pose = *getInput<geometry_msgs::PoseStamped>("robot_pose");
    geometry_msgs::PoseStamped target_pose = *getInput<geometry_msgs::PoseStamped>("target_pose");
    geometry_msgs::PoseStamped target_pose_cannon_rf;
    if (ttk::TF2::instance().transformPose("cannon_shaft_link", target_pose, target_pose_cannon_rf, ros::Duration(0.1)))
    {
      double adjacent = target_pose_cannon_rf.pose.position.x;
      double opposite = target_pose_cannon_rf.pose.position.z;
      float aim_angle = ttk::toDeg(std::atan(opposite / adjacent));
      float tilt_angle = std::max(std::min(aim_angle, +18.0f), -18.0f);
      ROS_INFO_NAMED(name(), "Target at %.1f degrees (clipped to %.1f)", aim_angle, tilt_angle);
      setOutput("angle", tilt_angle);
      return BT::NodeStatus::SUCCESS;
    }
    ROS_ERROR_STREAM_NAMED(name(), "Unable to transform target pose into 'cannon_shaft_link' frame");
    return BT::NodeStatus::FAILURE;
  }
};

class CannonCmd : public BT::RosServiceNode<thorp_msgs::CannonCmd>
{
public:
  CannonCmd(const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode<thorp_msgs::CannonCmd>(node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    // overwrite service_name with a default value
    return { BT::InputPort<std::string>("service_name", "cannon_command", "name of the ROS service") };
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (response.error.code == thorp_msgs::ThorpError::SUCCESS)
      return BT::NodeStatus::SUCCESS;

    ROS_ERROR_STREAM(name() << ": " << response.error.text);
    return BT::NodeStatus::FAILURE;
  }
};

class TiltCannon : public CannonCmd
{
public:
  TiltCannon(const std::string& node_name, const BT::NodeConfiguration& conf) : CannonCmd(node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<float>("angle") };
  }

  void sendRequest(RequestType& request) override
  {
    request.action = thorp_msgs::CannonCmdRequest::TILT;
    request.angle = *getInput<float>("angle");
  }
};

class FireCannon : public CannonCmd
{
public:
  FireCannon(const std::string& node_name, const BT::NodeConfiguration& conf) : CannonCmd(node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<uint32_t>("shots") };
  }

  void sendRequest(RequestType& request) override
  {
    request.action = thorp_msgs::CannonCmdRequest::FIRE;
    request.shots = *getInput<uint32_t>("shots");
  }
};

}  // namespace thorp::bt::actions
