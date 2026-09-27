#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

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
  GetClosestPose(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses"),  //
             BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),         //
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("closest_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto poses = requireInput<std::vector<geometry_msgs::msg::PoseStamped>>(*this, "poses");
    if (poses.empty())
    {
      RCLCPP_WARN_STREAM(logger(*this), "Pose list is empty");
      return BT::NodeStatus::FAILURE;
    }

    // Find the closest pose to the target one
    const auto target_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "target_pose");
    geometry_msgs::msg::PoseStamped closest_pose;
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