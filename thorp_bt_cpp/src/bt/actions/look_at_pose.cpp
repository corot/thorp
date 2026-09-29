#include <cmath>

#include <nav2_behavior_tree/bt_action_node.hpp>

#include "thorp_bt_cpp/node_common.hpp"

#include <nav2_msgs/action/spin.hpp>

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Turn on the spot to face a pose, without translating, with Nav2's spin behavior. To point the camera, fixed to the
 * base.
 */
class LookAtPose : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Spin>
{
public:
  LookAtPose(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : BtActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),   //
                                BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),  //
                                BT::OutputPort<int>("error") });
  }

private:
  void on_tick() override
  {
    const auto robot_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "robot_pose");
    auto target_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "target_pose");
    if (!ttk::TF2::instance().transformPose(robot_pose.header.frame_id, target_pose, target_pose))
      throw BT::RuntimeError(name(), ": cannot transform target pose to ", robot_pose.header.frame_id);

    const double heading = std::atan2(target_pose.pose.position.y - robot_pose.pose.position.y,
                                      target_pose.pose.position.x - robot_pose.pose.position.x);
    goal_ = nav2_msgs::action::Spin::Goal();
    goal_.target_yaw = static_cast<float>(ttk::normAngle(heading - ttk::yaw(robot_pose)));
    goal_.time_allowance.sec = 10;
  }

  BT::NodeStatus on_aborted() override
  {
    RCLCPP_ERROR(logger(*this), "Turning to face the target pose failed with error %d: %s",
                 result_.result->error_code, result_.result->error_msg.c_str());
    setOutput<int>("error", result_.result->error_code);
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_ACTION_NODE(LookAtPose, "spin");
};

}  // namespace thorp::bt::actions
