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
    return { BT::InputPort<std::vector<moveit_msgs::CollisionObject>>("objects"),  //
             BT::InputPort<uint32_t>("given_up_count"),                            //
             BT::InputPort<bool>("ignore_given_up") };
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

    auto ignore_given_up = getInput<bool>("ignore_given_up");
    if (ignore_given_up && *ignore_given_up)
    {
      auto given_up_count = getInput<uint32_t>("given_up_count");
      if (given_up_count && *given_up_count == objects->size())
      {
        ROS_INFO_STREAM_NAMED(name(), "All " << objects->size() << " detected objects have been given up");
        return BT::NodeStatus::FAILURE;
      }
    }

    ROS_INFO_STREAM_NAMED(name(), objects->size() << " objects available");
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ObjectsDetected);
};
}  // namespace thorp::bt::conditions
