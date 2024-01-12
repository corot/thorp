#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/alternative_config.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/** Use an alternative configuration while running */
class UseNamedConfig : public BT::StatefulActionNode
{
public:
  UseNamedConfig(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
    config_name_ = *getInput<std::string>("config_name");
    auto source_ns = *getInput<std::string>("source_ns") + "/" + config_name_;
    auto target_ns = *getInput<std::string>("target_ns");
    config_ = std::make_unique<ttk::AlternativeConfig>(source_ns, target_ns, "default", config_name_);
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("config_name"),
             BT::InputPort<std::string>("source_ns"),
             BT::InputPort<std::string>("target_ns") };
  }

private:
  BT::NodeStatus onStart() override
  {
    if (config_ && !config_->setConfig(config_name_))
    {
      ROS_ERROR_STREAM_NAMED(name(), "Set " << config_name_ << " named configuration failed");
      return BT::NodeStatus::FAILURE;
    }
    ROS_INFO_STREAM_NAMED(name(), "Using " << config_name_ << " named configuration");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // Nothing to do here; just wait until someone stop this action
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (config_ && !config_->setConfig("default"))
    {
      ROS_ERROR_STREAM_NAMED(name(), "Restore default configuration failed");
    }
  }

  std::string config_name_;
  std::unique_ptr<ttk::AlternativeConfig> config_;

  BT_REGISTER_NODE(UseNamedConfig);
};
}  // namespace thorp::bt::actions
