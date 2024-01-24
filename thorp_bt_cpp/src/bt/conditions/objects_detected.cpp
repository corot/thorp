#include <behaviortree_cpp_v3/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <moveit_msgs/CollisionObject.h>

namespace thorp::bt::conditions
{
class ObjectsDetected : public BT::ConditionNode
{
public:
  ObjectsDetected(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<moveit_msgs::CollisionObject>>("objects") };
  }

private:
  BT::NodeStatus tick() override
  {
    auto objects = getInput<std::vector<moveit_msgs::CollisionObject>>("objects");
    if (!objects || objects->empty())
    {
      ROS_INFO_STREAM_NAMED(name(), "No objects available");
      return BT::NodeStatus::FAILURE;
    }
    ROS_INFO_STREAM_NAMED(name(), objects->size() << " objects available");
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ObjectsDetected);
};
}  // namespace thorp::bt::conditions
