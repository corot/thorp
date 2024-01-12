#include <behaviortree_cpp_v3/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_msgs/FollowPoseFeedback.h>

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
             BT::InputPort<thorp_msgs::FollowPoseFeedback>("follow_feedback") };
  }

private:
  BT::NodeStatus tick() override
  {
    float max_dist = *getInput<float>("max_dist");
    float max_angle = *getInput<float>("max_angle");
    geometry_msgs::PoseStamped robot_pose = *getInput<geometry_msgs::PoseStamped>("robot_pose");
    auto follow_feedback = getInput<thorp_msgs::FollowPoseFeedback>("follow_feedback");
    if (!follow_feedback)
    {
      ROS_INFO_THROTTLE_NAMED(1, name(), "Follow pose feedback not available");
      return BT::NodeStatus::FAILURE;
    }
    double dist_to_target = follow_feedback->dist_to_target;
    double angle_to_target = follow_feedback->angle_to_target;
    if (dist_to_target <= max_dist && std::abs(angle_to_target) <= max_angle)
    {
      ROS_INFO_NAMED(name(), "Target at %.2f m and %.2f rad is reachable!", dist_to_target, angle_to_target);
      return BT::NodeStatus::SUCCESS;
    }

    ROS_INFO_NAMED(name(), "Target at %.2f m and %.2f rad not reachable", dist_to_target, angle_to_target);
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(TargetReachable);
};
}  // namespace thorp::bt::conditions
