#include <behaviortree_cpp_v3/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <geometry_msgs/PoseStamped.h>

namespace thorp::bt::conditions
{
/** Check if a key exists the blackboard. */
template <typename T>
class InBlackboard : public BT::ConditionNode
{
public:
  InBlackboard(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<T>("key") };
  }

private:
  virtual BT::NodeStatus tick() override
  {
    return getInput<T>("key") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  static NodeRegister<InBlackboard<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(InBlackboard<geometry_msgs::PoseStamped>, "PoseInBlackboard");
}  // namespace thorp::bt::conditions 
