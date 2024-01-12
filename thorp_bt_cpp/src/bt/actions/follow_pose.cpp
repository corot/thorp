#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/action_client_node.hpp"

#include <thorp_msgs/FollowPoseAction.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class FollowPose : public BT::SimpleActionClientNode<thorp_msgs::FollowPoseAction>
{
public:
  FollowPose(const std::string& name, const BT::NodeConfiguration& config) : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::SimpleActionClientNode<thorp_msgs::FollowPoseAction>::providedPorts();
    ports["action_name"].setDefaultValue("pose_follower/follow");
    ports.insert({ BT::InputPort<float>("time_limit"),
                   BT::InputPort<float>("distance"),
                   BT::InputPort<bool>("stop_at_distance"),
                   BT::OutputPort<uint8_t>("error"),
                   BT::OutputPort<thorp_msgs::FollowPoseFeedback>("feedback") });
    return ports;
  }

  bool setGoal(GoalType& goal) override
  {
    goal.time_limit.fromSec(*getInput<float>("time_limit"));
    goal.distance = *getInput<float>("distance");
    goal.stop_at_distance = *getInput<bool>("stop_at_distance");
    return true;
  }

  void onFeedback(const thorp_msgs::FollowPoseFeedbackConstPtr& feedback) override
  {
    setOutput("feedback", *feedback);
  }

  BT::NodeStatus onAborted(const thorp_msgs::FollowPoseResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "FollowPose failed with error %d", res->outcome);

// TODO add message field    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }

private:
  BT_REGISTER_NODE(FollowPose);
};
}  // namespace thorp::bt::actions
