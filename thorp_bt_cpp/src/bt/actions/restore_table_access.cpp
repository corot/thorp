#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

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
  RestoreTableAccess(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    // detect_table names every table it finds "table", so that is what ClearTableAccess cleared
    return { BT::InputPort<std::string>("table_name", "table", "name of the table to restore access to") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto table_name = requireInput<std::string>(*this, "table_name") + " approach";
    tcl::ServiceClient::instance().removeObject(table_name, "free_space", "local");
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(RestoreTableAccess);
};
}  // namespace thorp::bt::actions
