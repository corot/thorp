#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <moveit_msgs/CollisionObject.h>
#include <thorp_toolkit/planning_scene.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::conditions
{
class ObjectsDetected : public BT::ConditionNode
{
public:
  ObjectsDetected(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
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
      ROS_INFO_STREAM_COND_NAMED(given_up_count && *given_up_count, name(), *given_up_count << " given up objects");
    }

    ROS_INFO_STREAM_NAMED(name(), objects->size() << " object(s) available: " << ttk::getIDs(*objects));
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ObjectsDetected);
};
}  // namespace thorp::bt::conditions
