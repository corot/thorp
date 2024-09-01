#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::conditions
{
class TargetReachable : public BT::ConditionNode
{
public:
  TargetReachable(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), tf2_(ttk::TF2::instance())
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<float>("max_dist"),
             BT::InputPort<float>("max_angle"),
             BT::InputPort<geometry_msgs::PoseStamped>("robot_pose"),
             BT::InputPort<geometry_msgs::PoseStamped>("target_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    float max_dist = *getInput<float>("max_dist");
    float max_angle = *getInput<float>("max_angle");
    auto robot_pose = getInput<geometry_msgs::PoseStamped>("robot_pose");
    auto target_pose = getInput<geometry_msgs::PoseStamped>("target_pose");
    if (!robot_pose || !target_pose)
    {
      ROS_INFO_THROTTLE_NAMED(1, name(), "Robot/target pose not available");
      return BT::NodeStatus::FAILURE;
    }

    // Transform both poses into the same reference frame
    if (!tf2_.transformPose(robot_pose->header.frame_id, *target_pose, *target_pose))
    {
      return BT::NodeStatus::FAILURE;
    }

    double dist_to_target = ttk::distance2D(*robot_pose, *target_pose);
    double angle_to_target = ttk::heading(*robot_pose, *target_pose);
    angle_to_target = ttk::anglesDiff(ttk::yaw(*robot_pose), angle_to_target); // make relative to robot yaw
    if (dist_to_target <= max_dist && std::abs(angle_to_target) <= max_angle)
    {
      ROS_INFO_NAMED(name(), "Target at %.2f m and %.2f rad reachable!", dist_to_target, angle_to_target);
      return BT::NodeStatus::SUCCESS;
    }

    ROS_INFO_NAMED(name(), "Target at %.2f m and %.2f rad not reachable", dist_to_target, angle_to_target);
    return BT::NodeStatus::FAILURE;
  }

  ttk::TF2& tf2_;

  BT_REGISTER_NODE(TargetReachable);
};
}  // namespace thorp::bt::conditions
