#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/planning_scene.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Remove collision object from the planning scene
 */
class RemoveObject : public BT::SyncActionNode
{
public:
  RemoveObject(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("object_name") };
  }

private:
  BT::NodeStatus tick() override
  {
    auto object_name = *getInput<std::string>("object_name");
    ROS_INFO_NAMED(name(), "Removing object '%s' from planning scene", object_name.c_str());
    ttk::PlanningScene::instance().removeObject(object_name);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(RemoveObject);
};
}  // namespace thorp::bt::actions
