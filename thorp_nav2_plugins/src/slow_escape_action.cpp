/*
 * Author: Jorge Santos
 */

#include <map>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <nav2_behavior_tree/bt_action_node.hpp>

#include "thorp_nav2_plugins/action/slow_escape.hpp"

namespace thorp::nav2_plugins
{
/**
 * Run the slow_escape behavior: down the cost gradient for a distance, out of collision or out to free space.
 */
class SlowEscapeAction : public nav2_behavior_tree::BtActionNode<thorp_nav2_plugins::action::SlowEscape>
{
  using Action = thorp_nav2_plugins::action::SlowEscape;

public:
  SlowEscapeAction(const std::string& name, const std::string& action_name, const BT::NodeConfiguration& config)
    : BtActionNode<Action>(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<std::string>("mode", "distance", "distance, out_of_collision or out_to_free_space"),
          BT::InputPort<double>("clearance", 0.0, "footprint padding to check whether it escaped, in meters"),
          BT::InputPort<double>("max_distance", 0.2, "distance to travel at most, in meters"),
          BT::InputPort<double>("time_allowance", 10.0, "time to escape in, in seconds"),
          BT::OutputPort<Action::Result::_error_code_type>("error_code_id", "the escape's error code") });
  }

  void on_tick() override
  {
    if (!BT::isStatusActive(status()))
    {
      static const std::map<std::string, uint8_t> MODES = { { "distance", Action::Goal::DISTANCE },
                                                            { "out_of_collision", Action::Goal::OUT_OF_COLLISION },
                                                            { "out_to_free_space", Action::Goal::OUT_TO_FREE_SPACE } };
      const std::string mode = getInput<std::string>("mode").value();
      const auto it = MODES.find(mode);
      if (it == MODES.end())
        throw BT::RuntimeError("Unknown escape mode: " + mode);
      goal_.mode = it->second;
      goal_.clearance = static_cast<float>(getInput<double>("clearance").value());
      goal_.max_distance = static_cast<float>(getInput<double>("max_distance").value());
      goal_.time_allowance = rclcpp::Duration::from_seconds(getInput<double>("time_allowance").value());
    }
    increment_recovery_count();
  }

  BT::NodeStatus on_success() override
  {
    setOutput("error_code_id", Action::Result::NONE);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus on_aborted() override
  {
    setOutput("error_code_id", result_.result->error_code);
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus on_cancelled() override
  {
    setOutput("error_code_id", Action::Result::NONE);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace thorp::nav2_plugins

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<thorp::nav2_plugins::SlowEscapeAction>(name, "slow_escape", config);
  };
  factory.registerBuilder<thorp::nav2_plugins::SlowEscapeAction>("SlowEscape", builder);
}
