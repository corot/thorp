#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <std_msgs/UInt16.h>

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
    sub_ = ros::NodeHandle().subscribe("cannon_ctrl/shots_left", 1, &CannonHasAmmo::callback, this);
  }

private:
  bool has_ammo_ = true;
  ros::Subscriber sub_;

  void callback(const std_msgs::UInt16& msg)
  {
    if (has_ammo_ = msg.data; !has_ammo_)
    {
      ROS_INFO_ONCE("Cannon ammunition exhausted");
    }
  }

  BT::NodeStatus tick() override
  {
    return has_ammo_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(CannonHasAmmo);
};
}  // namespace thorp::bt::conditions
