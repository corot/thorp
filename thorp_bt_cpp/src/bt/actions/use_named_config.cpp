#include <behaviortree_cpp/action_node.h>

#include <thorp_msgs/srv/set_named_config.hpp>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/service_call.hpp"

namespace thorp::bt::actions
{
/**
 * Use a named configuration while running, restoring the default one when halted; so it runs in parallel with the
 * actions that need it.
 */
class UseNamedConfig : public BT::StatefulActionNode
{
public:
  UseNamedConfig(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("config_name") };
  }

private:
  BT::NodeStatus onStart() override
  {
    return setConfig(requireInput<std::string>(*this, "config_name")) ? BT::NodeStatus::RUNNING
                                                                       : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onRunning() override
  {
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (rclcpp::ok())
    {
      setConfig("");
    }
  }

  bool setConfig(const std::string& config_name)
  {
    thorp_msgs::srv::SetNamedConfig::Request request;
    request.name = config_name;
    const auto response =
        callService<thorp_msgs::srv::SetNamedConfig>(rosNode(*this), "/named_configs/set", request);
    if (response && !response->success)
    {
      RCLCPP_ERROR(logger(*this), "%s", response->message.c_str());
    }
    return response && response->success;
  }

  BT_REGISTER_NODE(UseNamedConfig);
};

}  // namespace thorp::bt::actions
