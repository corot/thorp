#include <behaviortree_cpp_v3/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <rail_manipulation_msgs/SegmentedObject.h>

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
  TableValidSize(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<rail_manipulation_msgs::SegmentedObject>("table") };
  }

private:
  BT::NodeStatus tick() override
  {
    ros::NodeHandle pnh("~");
    const double table_min_side = pnh.param("table_min_side", 0.0);
    const double table_max_side = pnh.param("table_max_side", 0.0);
    if (!table_min_side && !table_max_side)
    {
      ROS_WARN_STREAM_ONCE_NAMED(name(), "No size limits set for tables");
      return BT::NodeStatus::SUCCESS;
    }

    const auto table = *getInput<rail_manipulation_msgs::SegmentedObject>("table");
    if (std::min(table.width, table.depth) < table_min_side ||
        std::max(table.width, table.depth) > table_max_side)
    {
      ROS_INFO_NAMED(name(), "Table detected at %s rejected due to invalid size: %.2f x %.2f",
                     ttk::toCStr2D(table.center), table.width, table.depth);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(TableValidSize);
};
}  // namespace thorp::bt::conditions
