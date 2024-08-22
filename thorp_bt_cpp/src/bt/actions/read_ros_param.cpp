#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

namespace thorp::bt::actions
{
/**
 * Read a ROS parameter from the private (strategy) namespace and store its value on the blackboard.
 *
 * @param[in]  param_name  Input parameter name.
 * @param[out] output_key  Blackboard key where the value will be stored.
 *
 * @return  SUCCESS if the parameter exists
 *          FAILURE otherwise
 */
template <typename T>
class ReadRosParam : public BT::SyncActionNode
{
public:
  ReadRosParam(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  , pnh_("~")
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("param_name"),  //
             BT::OutputPort<T>("output_key") };
  }

private:
  ros::NodeHandle pnh_;

  BT::NodeStatus tick() override
  {
    const auto param_name = *getInput<std::string>("param_name");
    T param_value;
    if (!pnh_.getParam(param_name, param_value))
    {
      ROS_ERROR_STREAM_NAMED(name(), "Parameter " << param_name << " not found in namespace " << pnh_.getNamespace());
      return BT::NodeStatus::FAILURE;
    }
    ROS_INFO_STREAM_NAMED(name(), "Read ROS parameter " << param_name << " with value " << param_value);
    setOutput("output_key", param_value);
    return BT::NodeStatus::SUCCESS;
  }

  static NodeRegister<ReadRosParam<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(ReadRosParam<int>, "ReadIntParam");
BT_REGISTER_TEMPLATE_NODE(ReadRosParam<bool>, "ReadBoolParam");
BT_REGISTER_TEMPLATE_NODE(ReadRosParam<float>, "ReadFloatParam");
BT_REGISTER_TEMPLATE_NODE(ReadRosParam<double>, "ReadDoubleParam");
BT_REGISTER_TEMPLATE_NODE(ReadRosParam<std::string>, "ReadStringParam");

}  // namespace thorp::bt::actions
