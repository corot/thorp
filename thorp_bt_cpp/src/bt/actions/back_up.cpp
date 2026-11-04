#include "thorp_bt_cpp/nav2_action_node.hpp"

#include <nav2_msgs/action/back_up.hpp>

namespace thorp::bt::actions
{
/**
 * Drive straight back with Nav2's backup behavior.
 */
class BackUp : public Nav2ActionNode<nav2_msgs::action::BackUp>
{
public:
  BackUp(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : Nav2ActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<double>("backup_dist", 0.15, "distance to back up (m)"),
                                BT::InputPort<double>("backup_speed", 0.025, "speed to back up at (m/s)"),
                                BT::InputPort<double>("time_allowance", 10.0, "time to back up within (s)") });
  }

private:
  Goal getGoal() override
  {
    Goal goal;
    goal.target.x = requireInput<double>(*this, "backup_dist");
    goal.speed = static_cast<float>(requireInput<double>(*this, "backup_speed"));
    goal.time_allowance = rclcpp::Duration::from_seconds(requireInput<double>(*this, "time_allowance"));
    return goal;
  }

  BT_REGISTER_ACTION_NODE(BackUp, "backup");
};

}  // namespace thorp::bt::actions
