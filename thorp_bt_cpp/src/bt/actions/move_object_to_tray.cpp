#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/planning_scene.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Move a collision object to the tray. The input pose z-coordinate is at tray's bottom;
 * we need to add half the object size, so it appears at the right height on planning scene.
 */
class MoveObjectToTray : public BT::SyncActionNode
{
public:
  MoveObjectToTray(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("object_name"),  //
             BT::InputPort<geometry_msgs::PoseStamped>("pose_on_tray") };
  }

private:
  BT::NodeStatus tick() override
  {
    auto& ps = ttk::PlanningScene::instance();
    auto object_name = *getInput<std::string>("object_name");
    auto pose_on_tray = *getInput<geometry_msgs::PoseStamped>("pose_on_tray");
    geometry_msgs::PoseStamped obj_pose;
    geometry_msgs::Vector3 obj_size;
    ps.getObjectData(object_name, obj_pose, obj_size); // TODO getObjectSize
    pose_on_tray.pose.position.z += obj_size.z / 2.0;
    ROS_INFO_NAMED(name(), "Object '%s' moved to tray at %s", object_name.c_str(), ttk::toCStr3D(pose_on_tray));
    ps.moveObjectToTray(object_name, pose_on_tray);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(MoveObjectToTray);
};
}  // namespace thorp::bt::actions
