#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"

#include <thorp_toolkit/tray.hpp>

namespace thorp::bt::actions
{
/**
 * Pose to place the object in the gripper on the next free slot of the tray. Placing poses are the gripper's, that
 * grasps objects by their top, so the pose is raised by the object's height.
 *
 * @return  SUCCESS with the pose, FAILURE if the tray is full
 */
class NextPoseOnTray : public BT::SyncActionNode
{
public:
  NextPoseOnTray(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_on_tray") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto free_slots = thorp::toolkit::Tray().freeSlots(planningScene().getObjects());
    if (free_slots.empty())
    {
      RCLCPP_ERROR(logger(*this), "Tray is full");
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped pose = free_slots.front();
    const auto attached_objects = planningScene().getAttachedObjects();
    if (!attached_objects.empty())
      pose.pose.position.z += objectHeight(attached_objects.begin()->second.object);
    setOutput("pose_on_tray", pose);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(NextPoseOnTray);
};

}  // namespace thorp::bt::actions
