#include "thorp_bt_cpp/ros_action_node.hpp"

#include "thorp_bt_cpp/node_common.hpp"

#include <control_msgs/action/parallel_gripper_command.hpp>

namespace thorp::bt::conditions
{
/**
 * Check whether the gripper holds something by closing it: it stalls short of closed if an object prevents it from
 * closing fully. A held object stays pressed with the closing force.
 */
class GripperBusy : public BT::RosActionNode<control_msgs::action::ParallelGripperCommand>
{
public:
  GripperBusy(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : RosActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<double>("closed_position", 0.45, "gripper_joint angle fully closed") });
  }

private:
  Goal getGoal() override
  {
    Goal goal;
    goal.command.name = { GRIPPER_JOINT };
    goal.command.position = { requireInput<double>(*this, "closed_position") };
    return goal;
  }

  BT::NodeStatus onSucceeded(const Result& result) override
  {
    if (result.state.position.empty())
    {
      RCLCPP_ERROR(logger(*this), "Closing gripper reported no position");
      return BT::NodeStatus::FAILURE;
    }
    const double position = result.state.position.front();
    // An empty gripper can also stall, around the closed position
    if (result.stalled && position < requireInput<double>(*this, "closed_position") - EMPTY_TOLERANCE)
    {
      RCLCPP_INFO(logger(*this), "Gripper holding something; stalled at %g", position);
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(logger(*this), "Gripper empty; closed to %g", position);
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onAborted(const Result& /*result*/) override
  {
    RCLCPP_ERROR(logger(*this), "Closing gripper failed");
    return BT::NodeStatus::FAILURE;
  }

  /** Closer to closed than this, the gripper holds nothing: only objects a few millimeters thin stall beyond it */
  static constexpr double EMPTY_TOLERANCE = 0.03;

  static constexpr const char* GRIPPER_JOINT = "gripper_joint";

  BT_REGISTER_ACTION_NODE(GripperBusy, "gripper_controller/gripper_cmd");
};

}  // namespace thorp::bt::conditions
