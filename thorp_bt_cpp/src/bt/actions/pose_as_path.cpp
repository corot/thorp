#include <cmath>

#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Create a nav_msgs/Path msg with a single pose, or a straight one from a start pose to it.
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
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),                                             //
      BT::InputPort<geometry_msgs::msg::PoseStamped>("start", "if given, a straight path from it to pose"),  //
      BT::OutputPort<nav_msgs::msg::Path>("path"),
    };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "pose");
    nav_msgs::msg::Path path;
    path.header = pose.header;
    if (auto start = getInput<geometry_msgs::msg::PoseStamped>("start"))
    {
      // Poses every STEP on the way, heading to the goal, as path followers need dense paths
      if (!ttk::TF2::instance().transformPose(pose.header.frame_id, *start, *start))
        throw BT::RuntimeError(name(), ": cannot transform start pose to ", pose.header.frame_id);
      const double length = ttk::distance2D(start->pose, pose.pose);
      const double heading = std::atan2(pose.pose.position.y - start->pose.position.y,
                                        pose.pose.position.x - start->pose.position.x);
      for (double d = 0.0; d < length; d += STEP)
        path.poses.push_back(ttk::createPose(start->pose.position.x + d * std::cos(heading),
                                             start->pose.position.y + d * std::sin(heading), heading,
                                             pose.header.frame_id));
    }
    path.poses.push_back(pose);
    setOutput("path", path);
    return BT::NodeStatus::SUCCESS;
  }

  static constexpr double STEP = 0.05;

  BT_REGISTER_NODE(PoseAsPath);
};
}  // namespace thorp::bt::actions
