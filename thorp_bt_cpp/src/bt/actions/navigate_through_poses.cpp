#include <vector>

#include "thorp_bt_cpp/nav2_action_node.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>

namespace thorp::bt::actions
{
/**
 * Navigate through a list of poses with Nav2's navigator, reaching only the last one's heading.
 */
class NavigateThroughPoses : public Nav2ActionNode<nav2_msgs::action::NavigateThroughPoses>
{
public:
  NavigateThroughPoses(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : Nav2ActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals", "poses to navigate through") });
  }

private:
  Goal getGoal() override
  {
    Goal goal;
    goal.poses.goals = requireInput<std::vector<geometry_msgs::msg::PoseStamped>>(*this, "goals");
    if (!goal.poses.goals.empty())
      goal.poses.header = goal.poses.goals.front().header;
    return goal;
  }

  BT_REGISTER_ACTION_NODE(NavigateThroughPoses, "navigate_through_poses");
};

}  // namespace thorp::bt::actions
