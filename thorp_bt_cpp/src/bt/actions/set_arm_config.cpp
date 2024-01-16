#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/action_client_node.hpp"

#include <thorp_msgs/MoveToTargetAction.h>

namespace thorp::bt::actions
{
/**
 * Move arm into one of the stored configuration (resting, right_up, etc.)
 */
class SetArmConfig : public BT::SimpleActionClientNode<thorp_msgs::MoveToTargetAction>
{
public:
  SetArmConfig(const std::string& name, const BT::NodeConfiguration& config) : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::SimpleActionClientNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("move_to_target");
    ports.insert({ BT::InputPort<std::string>("configuration"), BT::OutputPort<unsigned int>("error"),
                   BT::OutputPort<std::optional<FeedbackType>>("feedback") });
    return ports;
  }

  bool setGoal(GoalType& goal) override
  {
    goal.target_type = GoalType::NAMED_TARGET;
    goal.named_target = *getInput<std::string>("configuration");
    return true;
  }

  void onFeedback(const FeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<FeedbackType>(*feedback));
  }

  BT::NodeStatus onAborted(const ResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "Error %d: %s", res->error.code, res->error.text.c_str());
    setOutput("error", res->error.code);

    return BT::NodeStatus::FAILURE;
  }

private:
  BT_REGISTER_NODE(SetArmConfig);
};
}  // namespace thorp::bt::actions
