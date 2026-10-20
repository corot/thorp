#include <cmath>

#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace thorp::bt::actions
{
/**
 * Choose, among poses around a table, the one on a side of it as the robot sees the table from where it stands:
 * nearest, farthest, leftmost or rightmost. Sides are told by their direction from the table's center, relative to
 * the robot's line of sight, so a side without a pose, as one on an obstacle, fails rather than yield a neighbor's.
 * All poses on the same frame.
 */
class GetTableSidePose : public BT::SyncActionNode
{
public:
  GetTableSidePose(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses", "poses around the table"),
             BT::InputPort<geometry_msgs::msg::PoseStamped>("table_pose"),  //
             BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),  //
             BT::InputPort<std::string>("side", "nearest, farthest, leftmost or rightmost"),
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("side_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto poses = requireInput<std::vector<geometry_msgs::msg::PoseStamped>>(*this, "poses");
    const auto table = requireInput<geometry_msgs::msg::PoseStamped>(*this, "table_pose").pose.position;
    const auto robot = requireInput<geometry_msgs::msg::PoseStamped>(*this, "robot_pose").pose.position;
    const auto side = requireInput<std::string>(*this, "side");

    // The line of sight from the robot to the table, and the direction the requested side lies in from its center
    const double sight = std::hypot(table.x - robot.x, table.y - robot.y);
    if (sight == 0.0)
      throw BT::RuntimeError(name(), ": the robot is on the table's center");
    const double sx = (table.x - robot.x) / sight, sy = (table.y - robot.y) / sight;
    double ux, uy;
    if (side == "nearest")
      ux = -sx, uy = -sy;
    else if (side == "farthest")
      ux = sx, uy = sy;
    else if (side == "leftmost")
      ux = -sy, uy = sx;
    else if (side == "rightmost")
      ux = sy, uy = -sx;
    else
      throw BT::RuntimeError(name(), ": unknown side '", side, "'; it can be nearest, farthest, leftmost or rightmost");

    const geometry_msgs::msg::PoseStamped* best = nullptr;
    double best_cosine = SIDE_MIN_COSINE;
    for (const auto& pose : poses)
    {
      const double dx = pose.pose.position.x - table.x, dy = pose.pose.position.y - table.y;
      const double distance = std::hypot(dx, dy);
      if (distance == 0.0)
        continue;
      const double cosine = (dx * ux + dy * uy) / distance;
      if (cosine > best_cosine)
      {
        best = &pose;
        best_cosine = cosine;
      }
    }
    if (!best)
    {
      RCLCPP_WARN(logger(*this), "No pose on the %s side of the table", side.c_str());
      return BT::NodeStatus::FAILURE;
    }
    setOutput("side_pose", *best);
    return BT::NodeStatus::SUCCESS;
  }

  /** Poses farther from a side's direction than 60 degrees are on another side, as sides are 90 degrees apart */
  static constexpr double SIDE_MIN_COSINE = 0.5;

  BT_REGISTER_NODE(GetTableSidePose);
};

}  // namespace thorp::bt::actions
