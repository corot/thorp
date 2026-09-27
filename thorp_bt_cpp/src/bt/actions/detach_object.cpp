#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"

namespace thorp::bt::actions
{
/**
 * Detach an object from the robot and remove it from the planning scene, as after releasing it off the table.
 */
class DetachObject : public BT::SyncActionNode
{
public:
  DetachObject(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
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
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.robot_state.is_diff = true;
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.object.id = object_name;
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene.robot_state.attached_collision_objects.push_back(attached);
    moveit_msgs::msg::CollisionObject object;
    object.id = object_name;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene.world.collision_objects.push_back(object);
    if (!planningScene().applyPlanningScene(scene))
    {
      RCLCPP_ERROR_STREAM(logger(*this), "Detach and remove " << object_name << " failed");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO_STREAM(logger(*this), object_name << " detached and removed");
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(DetachObject);
};

}  // namespace thorp::bt::actions
