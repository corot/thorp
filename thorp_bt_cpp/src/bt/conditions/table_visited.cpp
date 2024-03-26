#include <behaviortree_cpp_v3/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <rail_manipulation_msgs/SegmentedObject.h>

#include <thorp_costmap_layers/srv_iface_client.hpp>
namespace tcl = thorp::costmap_layers;

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::conditions
{
/**
 * Check whether the given table has been already visited. Returns:
 * SUCCESS if so
 * FAILURE otherwise
 */
class TableVisited : public BT::ConditionNode
{
public:
  TableVisited(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<rail_manipulation_msgs::SegmentedObject>("table"),  //
             BT::InputPort<geometry_msgs::PoseStamped>("table_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto table = *getInput<rail_manipulation_msgs::SegmentedObject>("table");
    const auto table_pose = *getInput<geometry_msgs::PoseStamped>("table_pose");
    geometry_msgs::Point lower_left, upper_right;
    lower_left.x = table_pose.pose.position.x - table.bounding_volume.dimensions.x / 2.0;
    lower_left.y = table_pose.pose.position.y - table.bounding_volume.dimensions.y / 2.0;
    upper_right.x = table_pose.pose.position.x + table.bounding_volume.dimensions.x / 2.0;
    upper_right.y = table_pose.pose.position.y + table.bounding_volume.dimensions.y / 2.0;
    const auto objects = tcl::ServiceClient::instance().queryObjects(lower_left, upper_right, "global");
    if (objects.empty())
    {
      ROS_DEBUG_NAMED(name(), "New table at %s with size %.2f x %.2f",
                      ttk::toCStr2D(table_pose), table.width, table.depth);
      return BT::NodeStatus::FAILURE;
    }
    ROS_DEBUG_NAMED(name(), "Table at %s with size %.2f x %.2f already visited",
                    ttk::toCStr2D(table_pose), table.width, table.depth);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(TableVisited);
};
}  // namespace thorp::bt::conditions
