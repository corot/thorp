#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <moveit_msgs/msg/collision_object.hpp>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::conditions
{
/**
 * Check whether the given table dimensions are within the expected limits. Returns:
 * SUCCESS if table dimensions are within limits
 * FAILURE otherwise
 */
class TableValidSize : public BT::ConditionNode
{
public:
  TableValidSize(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<moveit_msgs::msg::CollisionObject>("table") };
  }

private:
  BT::NodeStatus tick() override
  {
    const double table_min_side = rosNode(*this)->get_parameter_or("table_min_side", 0.0);
    const double table_max_side = rosNode(*this)->get_parameter_or("table_max_side", 0.0);
    if (!table_min_side && !table_max_side)
    {
      RCLCPP_WARN_ONCE(logger(*this), "No size limits set for tables");
      return BT::NodeStatus::SUCCESS;
    }

    const auto table = requireInput<moveit_msgs::msg::CollisionObject>(*this, "table");
    const double width = table.primitives.at(0).dimensions.at(1);
    const double depth = table.primitives.at(0).dimensions.at(0);
    if (std::min(width, depth) < table_min_side || std::max(width, depth) > table_max_side)
    {
      RCLCPP_INFO(logger(*this), "Table rejected due to invalid size: %.2f x %.2f", width, depth);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(TableValidSize);
};
}  // namespace thorp::bt::conditions
