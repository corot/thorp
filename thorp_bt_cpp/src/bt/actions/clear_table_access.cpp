#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <rail_manipulation_msgs/SegmentedObject.h>

#include <thorp_costmap_layers/srv_iface_client.hpp>
namespace tcl = thorp::costmap_layers;

namespace thorp::bt::actions
{
/**
 * Clear an area on the local costmap so the robot can approach the table
 */
class ClearTableAccess : public BT::SyncActionNode
{
public:
  ClearTableAccess(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
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
    const auto table_name = table.name + " approach";
    geometry_msgs::Vector3 table_size;
    table_size.x = 1.0;
    table_size.y = 0.5;
    tcl::ServiceClient::instance().addObject(table_name, "free_space", table_pose, table_size, "local");
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ClearTableAccess);
};
}  // namespace thorp::bt::actions
