#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
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
    return { BT::InputPort<std::vector<moveit_msgs::msg::CollisionObject>>("objects"),  //
             BT::InputPort<uint32_t>("given_up_count"),                            //
             BT::InputPort<bool>("ignore_given_up") };
  }

private:
  BT::NodeStatus tick() override
  {
    auto objects = getInput<std::vector<moveit_msgs::msg::CollisionObject>>("objects");
    if (!objects || objects->empty())
    {
      RCLCPP_INFO_STREAM(logger(*this), "No objects available");
      return BT::NodeStatus::FAILURE;
    }

    auto ignore_given_up = getInput<bool>("ignore_given_up");
    if (ignore_given_up && *ignore_given_up)
    {
      auto given_up_count = getInput<uint32_t>("given_up_count");
      if (given_up_count && *given_up_count == objects->size())
      {
        RCLCPP_INFO_STREAM(logger(*this), "All " << objects->size() << " detected objects have been given up");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO_STREAM_EXPRESSION(logger(*this), given_up_count && *given_up_count,
                                    *given_up_count << " given up objects");
    }

    RCLCPP_INFO_STREAM(logger(*this), objects->size() << " object(s) available: " << objectIds(*objects));
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ObjectsDetected);
};
}  // namespace thorp::bt::conditions
