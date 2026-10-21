#include <cmath>

#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace thorp::bt::actions
{
/**
 * Choose, among poses around a table, the one on a side of it named by compass on the map: north, south, east or
 * west, being +y, -y, +x and -x. A pose is on the side its direction from the table's center is nearest to, so each
 * pose is on a single side, and a side without one, as one on an obstacle, fails rather than yield a neighbor's.
 * All poses on the map frame.
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
             BT::InputPort<std::string>("side", "north, south, east or west"),
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("side_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto poses = requireInput<std::vector<geometry_msgs::msg::PoseStamped>>(*this, "poses");
    const auto table_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "table_pose");
    const auto side = requireInput<std::string>(*this, "side");
    if (table_pose.header.frame_id != "map")
      throw BT::RuntimeError(name(), ": the table pose is on ", table_pose.header.frame_id,
                             ", but sides are named on map");

    double side_angle;
    if (side == "east")
      side_angle = 0.0;
    else if (side == "north")
      side_angle = M_PI / 2.0;
    else if (side == "west")
      side_angle = M_PI;
    else if (side == "south")
      side_angle = -M_PI / 2.0;
    else
      throw BT::RuntimeError(name(), ": unknown side '", side, "'; it can be north, south, east or west");

    const auto& table = table_pose.pose.position;
    const geometry_msgs::msg::PoseStamped* best = nullptr;
    double best_offset = M_PI / 4.0;  // half the angle between sides: nearer to this side than to any other
    for (const auto& pose : poses)
    {
      const double dx = pose.pose.position.x - table.x, dy = pose.pose.position.y - table.y;
      if (dx == 0.0 && dy == 0.0)
        continue;
      const double offset = std::abs(std::remainder(std::atan2(dy, dx) - side_angle, 2.0 * M_PI));
      if (offset < best_offset)
      {
        best = &pose;
        best_offset = offset;
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

  BT_REGISTER_NODE(GetTableSidePose);
};

}  // namespace thorp::bt::actions
