#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_toolkit/tf2.hpp>
#include <thorp_toolkit/geometry.hpp>
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
             BT::InputPort<geometry_msgs::PoseStamped>("target_pose"), BT::OutputPort<float>("angle") };
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

  BT_REGISTER_NODE(AimCannon);
};

class CannonCmd : public BT::RosServiceNode<thorp_msgs::CannonCmd>
{
public:
  CannonCmd(const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode<ServiceType>(node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    // overwrite service_name with a default value
    BT::PortsList ports = BT::RosServiceNode<ServiceType>::providedPorts();
    ports["service_name"].setDefaultValue("cannon_command");
    return ports;
  }

private:
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
    BT::PortsList ports = CannonCmd::providedPorts();
    ports.insert({ BT::InputPort<float>("angle") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.action = ServiceType::Request::TILT;
    request.angle = *getInput<float>("angle");
  }

  BT_REGISTER_NODE(TiltCannon);
};

class FireCannon : public CannonCmd
{
public:
  FireCannon(const std::string& node_name, const BT::NodeConfiguration& conf) : CannonCmd(node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = CannonCmd::providedPorts();
    ports.insert({ BT::InputPort<uint32_t>("shots") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.action = ServiceType::Request::FIRE;
    request.shots = *getInput<uint32_t>("shots");
  }

  BT_REGISTER_NODE(FireCannon);
};

}  // namespace thorp::bt::actions
