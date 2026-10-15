#include <nav2_behavior_tree/bt_action_node.hpp>

#include "thorp_bt_cpp/errors.hpp"
#include "thorp_bt_cpp/node_common.hpp"

#include <nav2_msgs/action/follow_object.hpp>

namespace thorp::bt::actions
{
/**
 * Follow an object at the following server's distance, with opennav_following, keeping the robot facing it. The
 * object's pose comes on a topic, or as a TF frame. It runs until halted, the object is lost or max_duration passes.
 */
class FollowObject : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowObject>
{
public:
  FollowObject(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : BtActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<std::string>("pose_topic", "target_object_pose", "Topic with the object's pose"),
          BT::InputPort<std::string>("tracked_frame", "", "Object's TF frame, if no pose topic is given"),
          BT::InputPort<double>("max_duration", 0.0, "Seconds to follow the object; forever if 0"),
          BT::OutputPort<thorp_msgs::msg::ThorpError>("error") });
  }

private:
  void on_tick() override
  {
    goal_.pose_topic = requireInput<std::string>(*this, "pose_topic");
    goal_.tracked_frame = requireInput<std::string>(*this, "tracked_frame");
    goal_.max_duration = rclcpp::Duration::from_seconds(requireInput<double>(*this, "max_duration"));
  }

  BT::NodeStatus on_aborted() override
  {
    const std::string text =
        "following the object failed" + (result_.result->error_msg.empty() ? "" : ": " + result_.result->error_msg);
    RCLCPP_ERROR(logger(*this), "%s (error %d)", text.c_str(), result_.result->error_code);
    setOutput("error", makeError(result_.result->error_code, text));
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_ACTION_NODE(FollowObject, "follow_object");
};

}  // namespace thorp::bt::actions
