#include <cmath>

#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::conditions
{
/**
 * Check whether the target is within the given distance and heading from the robot.
 */
class TargetReachable : public BT::ConditionNode
{
public:
  TargetReachable(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("max_dist"),                               //
             BT::InputPort<double>("max_angle"),                              //
             BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),   //
             BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    const double max_dist = requireInput<double>(*this, "max_dist");
    const double max_angle = requireInput<double>(*this, "max_angle");
    const auto robot_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "robot_pose");
    auto target_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "target_pose");

    // Transform both poses into the same reference frame
    if (!ttk::TF2::instance().transformPose(robot_pose.header.frame_id, target_pose, target_pose,
                                            tf2::durationFromSec(0.1)))
    {
      RCLCPP_ERROR(logger(*this), "Unable to transform target pose into '%s' frame",
                   robot_pose.header.frame_id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    const double dist_to_target = ttk::distance2D(robot_pose, target_pose);
    const double angle_to_target = ttk::anglesDiff(ttk::yaw(robot_pose), ttk::heading(robot_pose, target_pose));
    if (dist_to_target <= max_dist && std::abs(angle_to_target) <= max_angle)
    {
      RCLCPP_INFO(logger(*this), "Target at %.2f m and %.2f rad reachable!", dist_to_target, angle_to_target);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG(logger(*this), "Target at %.2f m and %.2f rad not reachable", dist_to_target, angle_to_target);
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(TargetReachable);
};
}  // namespace thorp::bt::conditions
