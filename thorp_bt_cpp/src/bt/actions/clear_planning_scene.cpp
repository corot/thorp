#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/planning_scene.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Clear the planning scene, optionally sparing the tray and its content
 */
class ClearPlanningScene : public BT::SyncActionNode
{
public:
  ClearPlanningScene(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<bool>("keep_tray") };
  }

private:
  BT::NodeStatus tick() override
  {
    auto keep_tray = *getInput<bool>("keep_tray");
    ROS_INFO_NAMED(name(), "Clearing planning scene %s", keep_tray ? "but keeping tray and its content" : "");
    ttk::PlanningScene::instance().removeAll(keep_tray);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ClearPlanningScene);
};
}  // namespace thorp::bt::actions
