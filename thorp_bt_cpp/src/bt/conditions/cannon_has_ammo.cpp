#include <atomic>

#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <std_msgs/msg/u_int16.hpp>

#include <thorp_toolkit/common.hpp>

namespace thorp::bt::conditions
{
/**
 * Check whether the cannon still has ammunition to fire. Returns:
 * SUCCESS if so
 * FAILURE otherwise
 */
class CannonHasAmmo : public BT::ConditionNode
{
public:
  CannonHasAmmo(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
    sub_ = rosNode(*this)->create_subscription<std_msgs::msg::UInt16>(
        "cannon_ctrl/shots_left", toolkit::LATCHED,
        [this](const std_msgs::msg::UInt16::ConstSharedPtr msg) { callback(*msg); });
  }

private:
  std::atomic<bool> has_ammo_ = true;  // bt_server receives messages on another thread than the one ticking the tree
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_;

  void callback(const std_msgs::msg::UInt16& msg)
  {
    if (has_ammo_ = msg.data; !has_ammo_)
    {
      RCLCPP_INFO_ONCE(logger(*this), "Cannon ammunition exhausted");
    }
  }

  BT::NodeStatus tick() override
  {
    return has_ammo_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(CannonHasAmmo);
};
}  // namespace thorp::bt::conditions
