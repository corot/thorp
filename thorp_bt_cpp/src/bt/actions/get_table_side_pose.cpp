#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace thorp::bt::actions
{
namespace
{
/// A table's sides, named by compass on the map, and the directions they face from its center
constexpr std::array<std::pair<const char*, double>, 4> SIDES{
  { { "east", 0.0 }, { "north", M_PI / 2.0 }, { "west", M_PI }, { "south", -M_PI / 2.0 } }
};

/**
 * How far, in radians, the direction from a table's center to a pose is from a side's. A pose is on the side this is
 * smallest for, so each pose is on a single side.
 */
double offset(const geometry_msgs::msg::PoseStamped& table, const geometry_msgs::msg::PoseStamped& pose,
              double side_angle)
{
  const double dx = pose.pose.position.x - table.pose.position.x, dy = pose.pose.position.y - table.pose.position.y;
  return std::abs(std::remainder(std::atan2(dy, dx) - side_angle, 2.0 * M_PI));
}

void requireMapFrame(const BT::TreeNode& node, const geometry_msgs::msg::PoseStamped& table)
{
  if (table.header.frame_id != "map")
    throw BT::RuntimeError(node.name(), ": the table pose is on ", table.header.frame_id,
                           ", but sides are named on map");
}
}  // namespace

/**
 * Choose, among poses around a table, the one on a side of it named by compass on the map: north, south, east or
 * west, being +y, -y, +x and -x. A pose is on the side its direction from the table's center is nearest to, and a
 * side without one, as one on an obstacle, fails rather than yield a neighbor's. All poses on the map frame.
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
    const auto table = requireInput<geometry_msgs::msg::PoseStamped>(*this, "table_pose");
    const auto side = requireInput<std::string>(*this, "side");
    requireMapFrame(*this, table);
    const auto named = std::find_if(SIDES.begin(), SIDES.end(), [&](const auto& s) { return side == s.first; });
    if (named == SIDES.end())
      throw BT::RuntimeError(name(), ": unknown side '", side, "'; it can be north, south, east or west");

    const geometry_msgs::msg::PoseStamped* best = nullptr;
    double best_offset = M_PI / 4.0;  // half the angle between sides: nearer to this side than to any other
    for (const auto& pose : poses)
    {
      if (const double pose_offset = offset(table, pose, named->second); pose_offset < best_offset)
      {
        best = &pose;
        best_offset = pose_offset;
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

/**
 * Name the side of a table a pose is on, by compass on the map, as GetTableSidePose does. Poses on the map frame.
 */
class GetTableSide : public BT::SyncActionNode
{
public:
  GetTableSide(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),  //
             BT::InputPort<geometry_msgs::msg::PoseStamped>("table_pose"),
             BT::OutputPort<std::string>("side", "north, south, east or west") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "pose");
    const auto table = requireInput<geometry_msgs::msg::PoseStamped>(*this, "table_pose");
    requireMapFrame(*this, table);
    const auto side = std::min_element(SIDES.begin(), SIDES.end(), [&](const auto& a, const auto& b)
                                       { return offset(table, pose, a.second) < offset(table, pose, b.second); });
    setOutput("side", std::string(side->first));
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(GetTableSide);
};

}  // namespace thorp::bt::actions
