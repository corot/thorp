#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <mbf_msgs/ExePathAction.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Turn toward a given pose
 */
class LookAtPose : public BT::RosActionNode<mbf_msgs::ExePathAction>
{
public:
  LookAtPose(const std::string& name, const BT::NodeConfiguration& config) : RosActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("move_base_flex/exe_path");
    ports.insert({ BT::InputPort<std::string>("controller"),                  //
                   BT::InputPort<geometry_msgs::PoseStamped>("robot_pose"),   //
                   BT::InputPort<geometry_msgs::PoseStamped>("target_pose"),  //
                   BT::OutputPort<unsigned int>("error"),                     //
                   BT::OutputPort<std::optional<FeedbackType>>("feedback") });
    return ports;
  }

private:
  std::optional<GoalType> getGoal() override
  {
    GoalType new_goal;
    new_goal.controller = *getInput<std::string>("controller");

    // Create a single-pose path with the current robot location but heading toward the target pose
    geometry_msgs::PoseStamped robot_pose = *getInput<geometry_msgs::PoseStamped>("robot_pose");
    geometry_msgs::PoseStamped target_pose = *getInput<geometry_msgs::PoseStamped>("target_pose");
    double heading = ttk::heading(robot_pose, target_pose);
    ttk::setYaw(robot_pose, heading);
    new_goal.path.poses.push_back(robot_pose);
    if (!current_goal_ || *current_goal_ != new_goal)
    {
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
  }

  BT::NodeStatus onAborted(const ResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "LookAtPose failed at %.2f, %.2f, %.2f; distance to goal: %.2f, angle to goal: %.2f",
                    res->final_pose.pose.position.x, res->final_pose.pose.position.y, ttk::yaw(res->final_pose),
                    res->dist_to_goal, res->angle_to_goal);

    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }

  std::optional<GoalType> current_goal_;

  BT_REGISTER_NODE(LookAtPose);
};
}  // namespace thorp::bt::actions
