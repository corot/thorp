#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace thorp::bt::actions
{
/**
 * Create a nav_msgs/Path msg with a single pose.
 * @return  SUCCESS always
 */
class PoseAsPath : public BT::SyncActionNode
{
public:
  PoseAsPath(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
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
    path.poses.push_back(requireInput<geometry_msgs::PoseStamped>(*this, "pose"));
    setOutput("path", path);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(PoseAsPath);
};
}  // namespace thorp::bt::actions
