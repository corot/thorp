#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Return the closest pose from a list to a target one.
 * Returns FAILURE if the list is empty, SUCCESS otherwise.
 */
class GetClosestPose : public BT::SyncActionNode
{
public:
  GetClosestPose(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<geometry_msgs::PoseStamped>>("poses"),  //
             BT::InputPort<geometry_msgs::PoseStamped>("target_pose"),         //
             BT::OutputPort<geometry_msgs::PoseStamped>("closest_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto poses = *getInput<std::vector<geometry_msgs::PoseStamped>>("poses");
    if (poses.empty())
    {
      ROS_WARN_STREAM_NAMED(name(), "Pose list is empty");
      return BT::NodeStatus::FAILURE;
    }

    // Find the closest pose to the target one
    const auto target_pose = *getInput<geometry_msgs::PoseStamped>("target_pose");
    geometry_msgs::PoseStamped closest_pose;
    double closest_dist = std::numeric_limits<double>::infinity();
    for (auto& pose : poses)
    {
      double dist = ttk::distance2D(pose, target_pose);
      if (dist < closest_dist)
      {
        closest_pose = pose;
        closest_dist = dist;
      }
    }
    setOutput("closest_pose", closest_pose);

    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(GetClosestPose);
};
}  // namespace thorp::bt::actions