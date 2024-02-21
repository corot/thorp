#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <rail_manipulation_msgs/SegmentedObject.h>

#include <thorp_costmap_layers/srv_iface_client.hpp>
namespace tcl = thorp::costmap_layers;

namespace thorp::bt::actions
{
/**
 * Mark table area as an obstacle on both local and global costmaps,
 * so robot doesn't collide with the (for him) invisible eaves.
 */
class TableAsObstacle : public BT::SyncActionNode
{
public:
  TableAsObstacle(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
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
    geometry_msgs::Vector3 table_size;
    table_size.x = table.depth;
    table_size.y = table.width;
    tcl::ServiceClient::instance().addObject(table.name, "obstacle", table_pose, table_size, "both");
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(TableAsObstacle);
};
}  // namespace thorp::bt::actions
