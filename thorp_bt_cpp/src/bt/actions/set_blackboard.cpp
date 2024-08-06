#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <geometry_msgs/PoseStamped.h>

namespace thorp::bt::actions
{
/** Adapted from BT::SetBlackboard to work with any type. */
template <typename T>
class SetBlackboard : public BT::SyncActionNode
{
public:
  SetBlackboard(const std::string& name, const BT::NodeConfig& config)
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
    setOutput("output", *value);
    return BT::NodeStatus::SUCCESS;
  }

  static NodeRegister<SetBlackboard<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(SetBlackboard<bool>, "SetBool");
BT_REGISTER_TEMPLATE_NODE(SetBlackboard<double>, "SetDouble");
BT_REGISTER_TEMPLATE_NODE(SetBlackboard<uint32_t>, "SetUnsignedInt");
BT_REGISTER_TEMPLATE_NODE(SetBlackboard<geometry_msgs::PoseStamped>, "SetPose");
}  // namespace thorp::bt::actions
