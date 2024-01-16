#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/planning_scene.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Displace a collision object in the planning scene
 */
class DisplaceObject : public BT::SyncActionNode
{
public:
  DisplaceObject(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("object_name"),  //
             BT::InputPort<geometry_msgs::PoseStamped>("new_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    auto object_name = *getInput<std::string>("object_name");
    auto new_pose = *getInput<geometry_msgs::PoseStamped>("new_pose");
    ROS_INFO_NAMED(name(), "Object '%s' pose readjusted to %s", object_name.c_str(), ttk::toCStr3D(new_pose));
    ttk::PlanningScene::instance().displaceObject(object_name, new_pose);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(DisplaceObject);
};
}  // namespace thorp::bt::actions
