#include <nav2_behavior_tree/bt_action_node.hpp>

#include "thorp_bt_cpp/node_common.hpp"

#include <control_msgs/action/gripper_command.hpp>

namespace thorp::bt::conditions
{
/**
 * Check whether the gripper holds something by closing it: it stalls short of closed if an object prevents it from
 * closing fully. A held object stays pressed with the closing force.
 */
class GripperBusy : public nav2_behavior_tree::BtActionNode<control_msgs::action::GripperCommand>
{
public:
  GripperBusy(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : BtActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<double>("closed_position", 0.45, "gripper_joint angle fully closed") });
  }

private:
  void on_tick() override
  {
    goal_.command.position = requireInput<double>(*this, "closed_position");
  }

  BT::NodeStatus on_success() override
  {
    const auto& result = *result_.result;
    // An empty gripper can also stall, around the closed position
    if (result.stalled && result.position < requireInput<double>(*this, "closed_position") - EMPTY_TOLERANCE)
    {
      RCLCPP_INFO(logger(*this), "Gripper holding something; stalled at %g", result.position);
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(logger(*this), "Gripper empty; closed to %g", result.position);
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus on_aborted() override
  {
    RCLCPP_ERROR(logger(*this), "Closing gripper failed");
    return BT::NodeStatus::FAILURE;
  }

  /** Closer to closed than this, the gripper holds nothing: only objects a few millimeters thin stall beyond it */
  static constexpr double EMPTY_TOLERANCE = 0.03;

  BT_REGISTER_ACTION_NODE(GripperBusy, "gripper_controller/gripper_cmd");
};

}  // namespace thorp::bt::conditions
