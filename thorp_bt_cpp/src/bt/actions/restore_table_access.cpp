#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/semantic_layer.hpp"

namespace thorp::bt::actions
{
/**
 * Undo ClearTableAccess, restoring the local costmap around the table.
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
    thorp_costmap_layers::msg::Object access;
    access.operation = thorp_costmap_layers::msg::Object::REMOVE;
    access.type = "free_space";
    access.name = requireInput<std::string>(*this, "table_name") + " approach";
    return updateSemanticLayer(rosNode(*this), "local", { access }) ? BT::NodeStatus::SUCCESS :
                                                                      BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(RestoreTableAccess);
};

}  // namespace thorp::bt::actions
