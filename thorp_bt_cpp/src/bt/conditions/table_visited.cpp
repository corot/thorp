#include <cmath>

#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/semantic_layer.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::conditions
{
/**
 * Check whether a table has been visited already: TableAsObstacle marked a table overlapping it on the global
 * costmap. Returns:
 * SUCCESS if so
 * FAILURE otherwise
 */
class TableVisited : public BT::ConditionNode
{
public:
  TableVisited(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<moveit_msgs::msg::CollisionObject>("table"),  //
             BT::InputPort<geometry_msgs::msg::PoseStamped>("table_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    using shape_msgs::msg::SolidPrimitive;
    const auto table = requireInput<moveit_msgs::msg::CollisionObject>(*this, "table");
    const auto table_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "table_pose");
    const double length = table.primitives.at(0).dimensions.at(SolidPrimitive::BOX_X);
    const double width = table.primitives.at(0).dimensions.at(SolidPrimitive::BOX_Y);
    // the table's bounding box on the map
    const double yaw = ttk::yaw(table_pose);
    const double half_x = (length * std::abs(std::cos(yaw)) + width * std::abs(std::sin(yaw))) / 2.0;
    const double half_y = (length * std::abs(std::sin(yaw)) + width * std::abs(std::cos(yaw))) / 2.0;
    geometry_msgs::msg::Point lower_left, upper_right;
    lower_left.x = table_pose.pose.position.x - half_x;
    lower_left.y = table_pose.pose.position.y - half_y;
    upper_right.x = table_pose.pose.position.x + half_x;
    upper_right.y = table_pose.pose.position.y + half_y;
    const auto objects = querySemanticLayer(rosNode(*this), "global", lower_left, upper_right);
    if (!objects)
      return BT::NodeStatus::FAILURE;
    for (const auto& object : *objects)
    {
      if (object.type == "obstacle")
      {
        RCLCPP_INFO(logger(*this), "Table at %s already visited, as %s", ttk::toCStr2D(table_pose),
                    object.name.c_str());
        return BT::NodeStatus::SUCCESS;
      }
    }
    RCLCPP_INFO(logger(*this), "New table at %s, of %.2f x %.2f", ttk::toCStr2D(table_pose), length, width);
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(TableVisited);
};
}  // namespace thorp::bt::conditions
