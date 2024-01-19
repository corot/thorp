#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <mbf_msgs/MoveBaseAction.h>

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/reconfigure.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class GoToPose : public BT::RosActionNode<mbf_msgs::MoveBaseAction>
{
public:
  GoToPose(const std::string& name, const BT::NodeConfiguration& config) : RosActionNode(name, config)
  {
    auto controller = getInput<std::string>("controller");
    if (controller)
      reconf_ = ttk::Reconfigure("move_base_flex/" + *controller);
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("move_base_flex/move_base");
    ports.insert({ BT::InputPort<std::string>("planner"),              //
                   BT::InputPort<std::string>("controller"),           //
                   BT::InputPort<double>("dist_tolerance"),            //
                   BT::InputPort<double>("angle_tolerance"),           //
                   BT::InputPort<geometry_msgs::PoseStamped>("pose"),  //
                   BT::OutputPort<unsigned int>("error"), BT::OutputPort<std::optional<FeedbackType>>("feedback") });
    return ports;
  }

  std::optional<GoalType> getGoal() override
  {
    GoalType new_goal;
    auto target_pose = getInput<geometry_msgs::PoseStamped>("pose");
    if (!target_pose)
      throw BT::RuntimeError(name(), ": pose not provided");
    new_goal.target_pose = *target_pose;
    auto planner = getInput<std::string>("planner");
    if (planner)
      new_goal.planner = *planner;
    auto controller = getInput<std::string>("controller");
    if (controller)
      new_goal.controller = *controller;

    if (!current_goal_ || *current_goal_ != new_goal)
    {
      if (reconf_)
      {
        auto dist_tolerance = getInput<double>("dist_tolerance");
        if (dist_tolerance)
          reconf_->addParam("xy_goal_tolerance", *dist_tolerance);
        auto angle_tolerance = getInput<double>("angle_tolerance");
        if (angle_tolerance)
          reconf_->addParam("yaw_goal_tolerance", *angle_tolerance);
        if ((dist_tolerance || angle_tolerance) && !reconf_->apply())
          ROS_WARN_NAMED(name(), "Reconfigure goal tolerances failed");
      }

      current_goal_ = new_goal;
      return current_goal_;
    }
    return std::nullopt;
  }

  void onFeedback(const FeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<FeedbackType>(*feedback));
  }

  void onFinished() override
  {
    current_goal_.reset();
    if (reconf_)
      reconf_->revert();
  }

  BT::NodeStatus onAborted(const ResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "MoveBase failed at %.2f, %.2f, %.2f; distance to goal: %.2f, angle to goal: %.2f",
                    res->final_pose.pose.position.x, res->final_pose.pose.position.y, ttk::yaw(res->final_pose),
                    res->dist_to_goal, res->angle_to_goal);

    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }

private:
  std::optional<GoalType> current_goal_;
  std::optional<ttk::Reconfigure> reconf_;

  BT_REGISTER_NODE(GoToPose);
};
}  // namespace thorp::bt::actions
