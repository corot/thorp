#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <nav2_msgs/action/follow_path.hpp>

#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class GetRobotPose : public BT::StatefulActionNode
{
public:
  GetRobotPose(const std::string& name, const BT::NodeConfig& config)
    : StatefulActionNode(name, config), tf2_(ttk::TF2::instance())
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("timeout"),  //
             BT::OutputPort<int>("error"),      //
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("robot_pose") };
  }

private:
  BT::NodeStatus onStart() override
  {
    timeout_ = tf2::durationFromSec(requireInput<double>(*this, "timeout"));
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.frame_id = "base_footprint";
    robot_pose.pose.orientation.w = 1.0;
    if (tf2_.transformPose("map", robot_pose, robot_pose, timeout_))
    {
      setOutput("robot_pose", robot_pose);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(logger(*this), "Could not get the current robot pose");
    setOutput<int>("error", nav2_msgs::action::FollowPath::Result::TF_ERROR);

    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
  }

  ttk::TF2& tf2_;
  tf2::Duration timeout_;

  BT_REGISTER_NODE(GetRobotPose);
};
}  // namespace thorp::bt::actions
