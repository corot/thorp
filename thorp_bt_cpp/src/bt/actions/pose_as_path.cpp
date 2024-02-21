#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace thorp::bt::actions
{
class PoseAsPath : public BT::SyncActionNode
{
public:
  PoseAsPath(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::PoseStamped>("pose"),  //
      BT::OutputPort<nav_msgs::Path>("path"),
    };
  }

private:
  BT::NodeStatus tick() override
  {
    nav_msgs::Path path;
    path.poses.push_back(*getInput<geometry_msgs::PoseStamped>("pose"));
    setOutput("path", path);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(PoseAsPath);
};
}  // namespace thorp::bt::actions
