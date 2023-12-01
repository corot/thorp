#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::conditions
{
class TargetReachable : public BT::ConditionNode
{
public:
  TargetReachable(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
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
    geometry_msgs::PoseStamped robot_pose = *getInput<geometry_msgs::PoseStamped>("robot_pose");
    geometry_msgs::PoseStamped target_pose = *getInput<geometry_msgs::PoseStamped>("target_pose");
    double dist_to_target = ttk::distance2D(robot_pose, target_pose);
    double angle_to_target = ttk::heading(robot_pose, target_pose);
    if (dist_to_target <= max_dist && std::abs(angle_to_target) <= max_angle)
    {
      ROS_INFO_NAMED(name(), "Target at %.2f m and %.2f rad is reachable!", dist_to_target, angle_to_target);
      return BT::NodeStatus::SUCCESS;
    }

    ROS_INFO_NAMED(name(), "Target at %.2f m and %.2f rad not reachable", dist_to_target, angle_to_target);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace thorp::bt::conditions
