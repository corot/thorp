#include <algorithm>
#include <cmath>

#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_msgs/srv/cannon_command.hpp>

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Calculate the cannon tilt angle, in degrees, to aim at the target, clipped to the cannon's range.
 */
class AimCannon : public BT::SyncActionNode
{
public:
  AimCannon(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),  //
             BT::OutputPort<float>("angle") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto target_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "target_pose");
    geometry_msgs::msg::PoseStamped target_pose_cannon_rf;
    if (!ttk::TF2::instance().transformPose("cannon_shaft_link", target_pose, target_pose_cannon_rf,
                                            tf2::durationFromSec(0.1)))
    {
      RCLCPP_ERROR(logger(*this), "Unable to transform target pose into 'cannon_shaft_link' frame");
      return BT::NodeStatus::FAILURE;
    }
    const double adjacent = target_pose_cannon_rf.pose.position.x;
    const double opposite = target_pose_cannon_rf.pose.position.z;
    const auto aim_angle = static_cast<float>(ttk::toDeg(std::atan(opposite / adjacent)));
    const float tilt_angle = std::clamp(aim_angle, -MAX_TILT, +MAX_TILT);
    RCLCPP_INFO(logger(*this), "Target at %.1f degrees (clipped to %.1f)", aim_angle, tilt_angle);
    setOutput("angle", tilt_angle);
    return BT::NodeStatus::SUCCESS;
  }

  static constexpr float MAX_TILT = 18.0f;

  BT_REGISTER_NODE(AimCannon);
};

class CannonCommand : public BT::RosServiceNode<thorp_msgs::srv::CannonCommand, BT::SyncActionNode>
{
public:
  CannonCommand(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("cannon_command");
    return ports;
  }

private:
  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (response.error.code == thorp_msgs::msg::ThorpError::SUCCESS)
      return BT::NodeStatus::SUCCESS;

    RCLCPP_ERROR(logger(*this), "%s", response.error.text.c_str());
    return BT::NodeStatus::FAILURE;
  }
};

class TiltCannon : public CannonCommand
{
public:
  TiltCannon(const std::string& name, const BT::NodeConfig& conf) : CannonCommand(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = CannonCommand::providedPorts();
    ports.insert(BT::InputPort<float>("angle", "Tilt angle, in degrees; 0 is horizontal"));
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.action = RequestType::TILT;
    request.angle = requireInput<float>(*this, "angle");
  }

  BT_REGISTER_NODE(TiltCannon);
};

class FireCannon : public CannonCommand
{
public:
  FireCannon(const std::string& name, const BT::NodeConfig& conf) : CannonCommand(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = CannonCommand::providedPorts();
    ports.insert(BT::InputPort<unsigned int>("shots", 1, "Shots to fire, up to 6"));
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.action = RequestType::FIRE;
    request.shots = requireInput<unsigned int>(*this, "shots");
  }

  BT_REGISTER_NODE(FireCannon);
};

}  // namespace thorp::bt::actions
