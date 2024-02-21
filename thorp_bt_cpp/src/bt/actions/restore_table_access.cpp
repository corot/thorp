#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <rail_manipulation_msgs/SegmentedObject.h>

#include <thorp_costmap_layers/srv_iface_client.hpp>
namespace tcl = thorp::costmap_layers;

namespace thorp::bt::actions
{
/**
 * Restore the area cleared to approach the table, so we don't collide with it after detaching
 */
class RestoreTableAccess : public BT::SyncActionNode
{
public:
  RestoreTableAccess(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<rail_manipulation_msgs::SegmentedObject>("table") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto table = *getInput<rail_manipulation_msgs::SegmentedObject>("table");
    const auto table_name = table.name + " approach";
    tcl::ServiceClient::instance().removeObject(table_name, "free_space", "local");
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(RestoreTableAccess);
};
}  // namespace thorp::bt::actions
