#include <behaviortree_cpp/action_node.h>

#include <thorp_toolkit/tray.hpp>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"

namespace thorp::bt::actions
{
/**
 * Let an object just placed on the tray fall onto it in the planning scene, as the real one does: the place leaves it
 * where the gripper released it, above the tray and tilted as it was held.
 */
class SettleOnTray : public BT::SyncActionNode
{
public:
  SettleOnTray(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("object_name") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto object_name = requireInput<std::string>(*this, "object_name");
    const auto objects = planningScene().getObjects({ object_name });
    if (objects.empty())
    {
      RCLCPP_ERROR_STREAM(logger(*this), object_name << " not found in the planning scene");
      return BT::NodeStatus::FAILURE;
    }
    const auto& object = objects.begin()->second;
    const auto pose = thorp::toolkit::Tray().restingPose(object, objectHeight(object));
    if (!pose)
    {
      RCLCPP_ERROR_STREAM(logger(*this), "Transforming " << object_name << "'s pose to the tray frame failed");
      return BT::NodeStatus::FAILURE;
    }

    moveit_msgs::msg::CollisionObject moved;
    moved.id = object_name;
    moved.header = pose->header;
    moved.pose = pose->pose;
    moved.operation = moveit_msgs::msg::CollisionObject::MOVE;
    if (!planningScene().applyCollisionObject(moved))
    {
      RCLCPP_ERROR_STREAM(logger(*this), "Moving " << object_name << " onto the tray failed");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger(*this), "%s settled on the tray at [%.3f, %.3f, %.3f]", object_name.c_str(),
                pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(SettleOnTray);
};

}  // namespace thorp::bt::actions
