#include <nav2_behavior_tree/bt_action_node.hpp>

#include "thorp_bt_cpp/node_common.hpp"

#include <control_msgs/action/gripper_command.hpp>

namespace thorp::bt::actions
{
/**
 * Move the gripper to the given gripper_joint angle; the default opens it fully.
 */
class MoveGripper : public nav2_behavior_tree::BtActionNode<control_msgs::action::GripperCommand>
{
public:
  MoveGripper(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : BtActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<double>("position", -0.36, "gripper_joint angle") });
  }

private:
  void on_tick() override
  {
    goal_.command.position = requireInput<double>(*this, "position");
  }

  BT::NodeStatus on_aborted() override
  {
    RCLCPP_ERROR(logger(*this), "Move gripper to %g failed", goal_.command.position);
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_ACTION_NODE(MoveGripper, "gripper_controller/gripper_cmd");
};

}  // namespace thorp::bt::actions
