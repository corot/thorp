#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <mbf_msgs/ExePathAction.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Execute a given path using MBF's exe_path action.
 * Allows updating the goal while running.
 */
class ExePath : public BT::RosActionNode<mbf_msgs::ExePathAction>
{
public:
  ExePath(const std::string& name, const BT::NodeConfig& config) : RosActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("move_base_flex/exe_path");
    ports.insert({ BT::InputPort<std::string>("controller"),  //
                   BT::InputPort<nav_msgs::Path>("path"),     //
                   BT::OutputPort<int>("error"),              //
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
    new_goal.controller = *getInput<std::string>("controller");
    new_goal.path = *getInput<nav_msgs::Path>("path");
    if (!current_goal_ || *current_goal_ != new_goal)
    {
      current_goal_ = new_goal;
      goal_updated_ = true;
    }
  }

  void onFeedback(const FeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<FeedbackType>(*feedback));
  }

  void onFinished() override
  {
    current_goal_.reset();
  }

  BT::NodeStatus onAborted(const ResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "ExePath failed at %.2f, %.2f, %.2f; distance to goal: %.2f, angle to goal: %.2f",
                    res->final_pose.pose.position.x, res->final_pose.pose.position.y, ttk::yaw(res->final_pose),
                    res->dist_to_goal, res->angle_to_goal);

    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput<int>("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(ExePath);
};
}  // namespace thorp::bt::actions
