#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/actions/service_client_node.hpp"

#include <thorp_msgs/CannonCmd.h>

namespace thorp::bt::actions
{
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

class AimCannon : public CannonCmd
{
public:
  AimCannon(const std::string& node_name, const BT::NodeConfiguration& conf)
    : CannonCmd(node_name, conf)
  {
  }
  void sendRequest(RequestType& request) override
  {
    request.action = thorp_msgs::CannonCmdRequest::AIM;
  }
};

class TiltCannon : public CannonCmd
{
public:
  TiltCannon(const std::string& node_name, const BT::NodeConfiguration& conf)
    : CannonCmd(node_name, conf)
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

private:
  double angle_;
};

class FireCannon : public CannonCmd
{
public:
  FireCannon(const std::string& node_name, const BT::NodeConfiguration& conf)
    : CannonCmd(node_name, conf)
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
