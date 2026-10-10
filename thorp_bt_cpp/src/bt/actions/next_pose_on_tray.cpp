#include <algorithm>

#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"

#include <thorp_toolkit/tray.hpp>

namespace thorp::bt::actions
{
/**
 * Pose to place the object in the gripper on the next free slot of the tray, or on the next one after a given slot, as
 * one where placing failed. Placing poses are the object's, so the slot pose is raised by half the object's height, to
 * its center.
 *
 * @return  SUCCESS with the pose, FAILURE if no free slot is left
 */
class NextPoseOnTray : public BT::SyncActionNode
{
public:
  NextPoseOnTray(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("after", "a pose on a slot, as this node outputs"),
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_on_tray") };
  }

private:
  BT::NodeStatus tick() override
  {
    auto free_slots = thorp::toolkit::Tray().freeSlots(planningScene().getObjects());
    const auto after = getInput<geometry_msgs::msg::PoseStamped>("after");
    if (after)
    {
      // Free slots come in filling order: by rows along y, and along x within each row
      const auto not_after = [&after](const geometry_msgs::msg::PoseStamped& slot)
      {
        const double dx = slot.pose.position.x - after->pose.position.x;
        const double dy = slot.pose.position.y - after->pose.position.y;
        return dy < -SAME_SLOT_TOLERANCE || (dy < SAME_SLOT_TOLERANCE && dx < SAME_SLOT_TOLERANCE);
      };
      free_slots.erase(std::remove_if(free_slots.begin(), free_slots.end(), not_after), free_slots.end());
    }
    if (free_slots.empty())
    {
      if (after)
        RCLCPP_ERROR(logger(*this), "No free slot left on the tray after [%.3f, %.3f]", after->pose.position.x,
                     after->pose.position.y);
      else
        RCLCPP_ERROR(logger(*this), "Tray is full");
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped pose = free_slots.front();
    const auto attached_objects = planningScene().getAttachedObjects();
    if (!attached_objects.empty())
      pose.pose.position.z += objectHeight(attached_objects.begin()->second.object) / 2.0;
    RCLCPP_INFO_EXPRESSION(logger(*this), after.has_value(), "Next free slot on the tray, at [%.3f, %.3f]",
                           pose.pose.position.x, pose.pose.position.y);
    setOutput("pose_on_tray", pose);
    return BT::NodeStatus::SUCCESS;
  }

  /// Slot poses come from the same computation, so this only absorbs rounding
  static constexpr double SAME_SLOT_TOLERANCE = 0.001;

  BT_REGISTER_NODE(NextPoseOnTray);
};

}  // namespace thorp::bt::actions
