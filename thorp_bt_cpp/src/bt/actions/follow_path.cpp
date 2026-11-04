#include "thorp_bt_cpp/nav2_action_node.hpp"

#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/path.hpp>

namespace thorp::bt::actions
{
/**
 * Follow a path with Nav2's controller server, with the given controller and goal checker plugins.
 */
class FollowPath : public Nav2ActionNode<nav2_msgs::action::FollowPath>
{
public:
  FollowPath(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : Nav2ActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<nav_msgs::msg::Path>("path", "path to follow"),
          BT::InputPort<std::string>("controller_id", "", "controller plugin; the default if empty"),
          BT::InputPort<std::string>("goal_checker_id", "", "goal checker plugin; the default if empty") });
  }

private:
  Goal getGoal() override
  {
    Goal goal;
    goal.path = requireInput<nav_msgs::msg::Path>(*this, "path");
    goal.controller_id = requireInput<std::string>(*this, "controller_id");
    goal.goal_checker_id = requireInput<std::string>(*this, "goal_checker_id");
    return goal;
  }

  BT_REGISTER_ACTION_NODE(FollowPath, "follow_path");
};

}  // namespace thorp::bt::actions
