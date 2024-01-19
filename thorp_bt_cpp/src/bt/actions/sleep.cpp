#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

namespace thorp::bt::actions
{
class Sleep : public BT::StatefulActionNode
{
public:
  Sleep(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("duration") };
  }

private:
  BT::NodeStatus onStart() override
  {
    start_time_ = ros::Time::now();
    duration_.fromSec(*getInput<double>("duration"));
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    return ros::Time::now() - start_time_ < duration_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
  }

  ros::Time start_time_;
  ros::Duration duration_;

  BT_REGISTER_NODE(Sleep);
};
}  // namespace thorp::bt::actions
