#include <behaviortree_cpp_v3/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <kobuki_msgs/BumperEvent.h>

namespace thorp::bt::conditions
{
/**
 * Check whether the cannon still has ammunition to fire. Returns:
 * SUCCESS if so
 * FAILURE otherwise
 */
class BumperPressed : public BT::ConditionNode
{
public:
  BumperPressed(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
  {
    sub_ = ros::NodeHandle().subscribe("mobile_base/events/bumper", 1, &BumperPressed::callback);
  }

private:
  inline static bool bumper_pressed_ = false;
  inline static ros::Subscriber sub_;

  inline static void callback(const kobuki_msgs::BumperEvent& msg)
  {
    if (bumper_pressed_ = msg.state == kobuki_msgs::BumperEvent::PRESSED; bumper_pressed_)
    {
      ROS_INFO_STREAM("bumper pressed " << msg.bumper);
    }
  }

  BT::NodeStatus tick() override
  {
    return bumper_pressed_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(BumperPressed);
};
}  // namespace thorp::bt::conditions
