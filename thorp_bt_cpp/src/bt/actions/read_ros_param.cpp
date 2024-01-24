#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

namespace thorp::bt::actions
{
/** Read a ROS parameter and store its value on the blackboard. */
template <typename T>
class ReadRosParam : public BT::SyncActionNode
{
public:
  ReadRosParam(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("key"),  //
             BT::InputPort<bool>("negate"),      //
             BT::OutputPort<T>("value") };
  }

private:
  BT::NodeStatus tick() override
  {
    ros::NodeHandle pnh("~");
    const auto key = *getInput<std::string>("key");
    T value;
    if (!pnh.getParam(key, value))
    {
      ROS_ERROR_STREAM_NAMED(name(), "Parameter " << key << " not found in namespace " << pnh.getNamespace());
      return BT::NodeStatus::FAILURE;
    }
    if (const auto negate = getInput<bool>("negate"); negate && *negate)
    {
      value = - value;
    }
    ROS_INFO_STREAM_NAMED(name(), "Read ROS parameter " << key << " with value " << value);
    setOutput("value", value);
    return BT::NodeStatus::SUCCESS;
  }

  static NodeRegister<ReadRosParam<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(ReadRosParam<int>, "ReadIntParam");
BT_REGISTER_TEMPLATE_NODE(ReadRosParam<bool>, "ReadBoolParam");
BT_REGISTER_TEMPLATE_NODE(ReadRosParam<double>, "ReadDoubleParam");
}  // namespace thorp::bt::actions
