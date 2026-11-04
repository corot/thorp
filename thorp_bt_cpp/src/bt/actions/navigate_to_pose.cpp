#include "thorp_bt_cpp/nav2_action_node.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace thorp::bt::actions
{
/**
 * Navigate to a pose with Nav2's navigator.
 */
class NavigateToPose : public Nav2ActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToPose(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : Nav2ActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "pose to navigate to") });
  }

private:
  Goal getGoal() override
  {
    Goal goal;
    goal.pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "goal");
    return goal;
  }

  BT_REGISTER_ACTION_NODE(NavigateToPose, "navigate_to_pose");
};

}  // namespace thorp::bt::actions
