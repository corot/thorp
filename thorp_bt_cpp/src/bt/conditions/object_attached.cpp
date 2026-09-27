#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"

namespace thorp::bt::conditions
{
/**
 * Check whether the planning scene has an object attached to the robot, and name it.
 */
class ObjectAttached : public BT::ConditionNode
{
public:
  ObjectAttached(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::string>("attached_object") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto attached_objects = planningScene().getAttachedObjects();
    if (attached_objects.empty())
    {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("attached_object", attached_objects.begin()->first);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ObjectAttached);
};

}  // namespace thorp::bt::conditions
