#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <thorp_msgs/MoveToTargetAction.h>

namespace thorp::bt::actions
{
/**
 * Move arm into one of the stored configuration (resting, right_up, etc.)
 * Allows updating the goal while running.
 */
class SetArmConfig : public BT::RosActionNode<thorp_msgs::MoveToTargetAction>
{
public:
  SetArmConfig(const std::string& name, const BT::NodeConfig& config) : RosActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("manipulation/move_to_target");
    ports.insert({ BT::InputPort<std::string>("configuration"),  //
                   BT::OutputPort<int>("error"),                 //
                   BT::OutputPort<std::optional<FeedbackType>>("feedback") });
    return ports;
  }

private:
  std::optional<GoalType> current_goal_;

  GoalType getGoal() override
  {
    return *current_goal_;
  }

  void onTick() override
  {
    GoalType new_goal;
    new_goal.target_type = GoalType::NAMED_TARGET;
    new_goal.named_target = *getInput<std::string>("configuration");
    if (!current_goal_ || *current_goal_ != new_goal)
    {
      current_goal_ = new_goal;
      goal_updated_ = true;
    }
  }

  void onFinished() override
  {
    current_goal_.reset();
  }

  void onFeedback(const FeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<FeedbackType>(*feedback));
  }

  BT::NodeStatus onAborted(const ResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "Error %d: %s", res->error.code, res->error.text.c_str());
    setOutput<int>("error", res->error.code);

    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(SetArmConfig);
};
}  // namespace thorp::bt::actions
