#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"
#include "thorp_bt_cpp/tray.hpp"

namespace thorp::bt::actions
{
/**
 * Remove all the objects from the planning scene, optionally keeping those on the tray.
 */
class ClearPlanningScene : public BT::SyncActionNode
{
public:
  ClearPlanningScene(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<bool>("keep_tray") };
  }

private:
  BT::NodeStatus tick() override
  {
    const bool keep_tray = requireInput<bool>(*this, "keep_tray");
    RCLCPP_INFO(logger(*this), "Clearing planning scene%s", keep_tray ? ", but keeping tray content" : "");
    std::vector<std::string> to_remove;
    Tray tray(rosNode(*this));
    for (const auto& [id, object] : planningScene().getObjects())
    {
      if (!keep_tray || !tray.onTray(object))
      {
        to_remove.push_back(id);
      }
    }
    planningScene().removeCollisionObjects(to_remove);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ClearPlanningScene);
};

}  // namespace thorp::bt::actions
