#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace thorp::bt::actions
{
/**
 * Publish a pose on a latched topic, to show it on RViz, as navigation goals: Nav2 keeps the goal on its own
 * blackboard. Not on goal_pose, that bt_navigator takes as a new goal.
 */
class PublishPose : public BT::SyncActionNode
{
public:
  PublishPose(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),  //
             BT::InputPort<std::string>("topic", "~/nav_goal", "topic to publish on") };
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::string topic_;

  BT::NodeStatus tick() override
  {
    const auto pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "pose");
    const auto topic = requireInput<std::string>(*this, "topic");
    if (!publisher_ || topic != topic_)
    {
      publisher_ =
          rosNode(*this)->create_publisher<geometry_msgs::msg::PoseStamped>(topic, rclcpp::QoS(1).transient_local());
      topic_ = topic;
    }
    publisher_->publish(pose);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(PublishPose);
};

}  // namespace thorp::bt::actions
