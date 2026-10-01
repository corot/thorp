#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"

#include <thorp_toolkit/tray.hpp>

namespace thorp::bt::conditions
{
/**
 * Check whether all the tray slots have an object on them.
 */
class TrayFull : public BT::ConditionNode
{
public:
  TrayFull(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  BT::NodeStatus tick() override
  {
    const bool tray_full = thorp::toolkit::Tray().freeSlots(planningScene().getObjects()).empty();
    RCLCPP_DEBUG_EXPRESSION(logger(*this), tray_full, "The tray is full");
    return tray_full ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(TrayFull);
};

}  // namespace thorp::bt::conditions
