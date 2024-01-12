#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

namespace thorp::bt::actions
{
/** Adapted from BT::SetBlackboard to work with any type. */
template <typename T>
class SetBlackboard : public BT::SyncActionNode
{
public:
  SetBlackboard(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<T>("input"), BT::OutputPort<T>("output") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto value = getInput<T>("input");
    if (!value)
    {
      ROS_ERROR_STREAM_NAMED(name(), "No value provided");
      return BT::NodeStatus::FAILURE;
    }
    ROS_ERROR_STREAM("SetBlackboard  " << *value);
    setOutput("output", *value);
    return BT::NodeStatus::SUCCESS;
  }

  static NodeRegister<SetBlackboard<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(SetBlackboard<bool>, "SetBool");
BT_REGISTER_TEMPLATE_NODE(SetBlackboard<double>, "SetDouble");
BT_REGISTER_TEMPLATE_NODE(SetBlackboard<uint32_t>, "SetUnsignedInt");
}  // namespace thorp::bt::actions
