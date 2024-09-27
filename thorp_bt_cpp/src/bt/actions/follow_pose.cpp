#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <thorp_msgs/FollowPoseAction.h>

namespace thorp::bt::actions
{
class FollowPose : public BT::RosActionNode<thorp_msgs::FollowPoseAction>
{
public:
  FollowPose(const std::string& name, const BT::NodeConfig& config) : RosActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("pose_follower/follow");
    ports.insert({ BT::InputPort<float>("no_pose_timeout"),  //
                   BT::InputPort<float>("exec_time_limit"),  //
                   BT::InputPort<float>("target_distance"),  //
                   BT::InputPort<bool>("stop_at_distance"),  //
                   BT::OutputPort<int>("error"),             //
                   BT::OutputPort<FeedbackType>("feedback") });
    return ports;
  }

private:
  GoalType getGoal() override
  {
    GoalType goal;
    goal.no_pose_timeout.fromSec(*getInput<float>("no_pose_timeout"));
    goal.exec_time_limit.fromSec(*getInput<float>("exec_time_limit"));
    goal.target_distance = *getInput<float>("target_distance");
    goal.stop_at_distance = *getInput<bool>("stop_at_distance");
    return goal;
  }

  void onFeedback(const FeedbackConstPtr& feedback) override
  {
    setOutput("feedback", *feedback);
  }

  BT::NodeStatus onAborted(const ResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "FollowPose failed with error %d", res->outcome);

    // TODO add message field    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput<int>("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(FollowPose);
};
}  // namespace thorp::bt::actions
