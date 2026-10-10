#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thorp_toolkit/tray.hpp>

namespace thorp::bt::conditions
{
/**
 * Check whether a pose is over the tray: within its sides, from slightly below its surface to well above it.
 */
class PoseOnTray : public BT::ConditionNode
{
public:
  PoseOnTray(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "pose");
    return thorp::toolkit::Tray().onTray(pose) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(PoseOnTray);
};

}  // namespace thorp::bt::conditions
