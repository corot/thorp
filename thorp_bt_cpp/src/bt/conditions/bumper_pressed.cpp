#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/kobuki_base.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::conditions
{
/**
 * Listen for bumper pressed events. Returns:
 * SUCCESS if an event has occurred (no time limit on when)
 * FAILURE otherwise
 */
class BumperPressed : public BT::ConditionNode
{
public:
  BumperPressed(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
    sub_ = ros::NodeHandle().subscribe("mobile_base/events/bumper", 1, &BumperPressed::callback, this);
  }

private:
  bool bumper_pressed_ = false;
  ros::Subscriber sub_;

  void callback(const kobuki_msgs::BumperEvent& msg)
  {
    if (bumper_pressed_ = msg.state == kobuki_msgs::BumperEvent::PRESSED; bumper_pressed_)
    {
      emitWakeUpSignal();  // trigger immediate reaction

      ROS_INFO_STREAM_NAMED(name(), ttk::bumperName(msg.bumper) << " bumper pressed");
    }
  }

  BT::NodeStatus tick() override
  {
    if (bumper_pressed_)
    {
      bumper_pressed_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(BumperPressed);
};
}  // namespace thorp::bt::conditions
