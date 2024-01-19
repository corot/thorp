#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <mbf_msgs/RecoveryAction.h>

namespace thorp::bt::actions
{
class Recovery : public BT::RosActionNode<mbf_msgs::RecoveryAction>
{
public:
  Recovery(const std::string& name, const BT::NodeConfiguration& config) : RosActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("move_base_flex/recovery");
    ports.insert({ BT::InputPort<std::string>("behavior"),  //
                   BT::OutputPort<std::optional<FeedbackType>>("feedback") });
    return ports;
  }

private:
  std::optional<GoalType> getGoal() override
  {
    if (status() == BT::NodeStatus::RUNNING)
      return std::nullopt;

    GoalType goal;
    goal.behavior = *getInput<std::string>("behavior");
    return goal;
  }

  void onFeedback(const FeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<FeedbackType>(*feedback));
  }

  BT::NodeStatus onAborted(const ResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(Recovery);
};
}  // namespace thorp::bt::actions
