#include "thorp_bt_cpp/ros_action_node.hpp"

#include "thorp_bt_cpp/node_common.hpp"

#include <control_msgs/action/parallel_gripper_command.hpp>

namespace thorp::bt::actions
{
/**
 * Move the gripper to the given gripper_joint angle; the default opens it fully.
 */
class MoveGripper : public BT::RosActionNode<control_msgs::action::ParallelGripperCommand>
{
public:
  MoveGripper(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : RosActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<double>("position", -0.36, "gripper_joint angle") });
  }

private:
  Goal getGoal() override
  {
    Goal goal;
    goal.command.name = { GRIPPER_JOINT };
    goal.command.position = { requireInput<double>(*this, "position") };
    return goal;
  }

  BT::NodeStatus onAborted(const Result& /*result*/) override
  {
    RCLCPP_ERROR(logger(*this), "Move gripper to %g failed", requireInput<double>(*this, "position"));
    return BT::NodeStatus::FAILURE;
  }

  static constexpr const char* GRIPPER_JOINT = "gripper_joint";

  BT_REGISTER_ACTION_NODE(MoveGripper, "gripper_controller/gripper_cmd");
};

}  // namespace thorp::bt::actions
