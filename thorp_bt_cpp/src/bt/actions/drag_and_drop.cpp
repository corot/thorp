#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <moveit_msgs/CollisionObject.h>
#include <thorp_msgs/DragAndDropAction.h>

namespace thorp::bt::actions
{
/**
 * Wait for user to drag and drop a tabletop object and return its name, pickup and place poses.
 */
class DragAndDrop : public BT::RosActionNode<thorp_msgs::DragAndDropAction>
{
public:
  DragAndDrop(const std::string& name, const BT::NodeConfiguration& config) : RosActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("drag_and_drop");
    ports.insert({ // BT::InputPort<std::string>("output_frame"),                           // TODO
                   BT::InputPort<std::vector<moveit_msgs::CollisionObject>>("objects"),  //
                                                                                         // BT::OutputPort<moveit_msgs::CollisionObject>("target_object"),
                                                                                         // //  TODO
                   BT::OutputPort<std::string>("object_name"),                 //
                   BT::OutputPort<geometry_msgs::PoseStamped>("pickup_pose"),  //
                   BT::OutputPort<geometry_msgs::PoseStamped>("place_pose"),   //
                   BT::OutputPort<std::optional<FeedbackType>>("feedback") });
    return ports;
  }

private:
  std::optional<GoalType> getGoal() override
  {
    if (status() == BT::NodeStatus::RUNNING)
      return std::nullopt;

    ros::NodeHandle pnh("~");

    GoalType goal;
    goal.output_frame = pnh.param("picking_planning_frame", std::string("arm_base_link"));
    const auto objects = *getInput<std::vector<moveit_msgs::CollisionObject>>("objects");
    std::for_each(objects.begin(), objects.end(), [&](const auto& o) { goal.object_names.push_back(o.id); });
    return goal;
  }

  BT::NodeStatus onSucceeded(const ResultConstPtr& res) override
  {
    ROS_INFO_STREAM_NAMED(name(), "Drag and drop " << res->object_name);
    setOutput("object_name", res->object_name);
    setOutput("pickup_pose", res->pickup_pose);
    setOutput("place_pose", res->place_pose);

    return BT::NodeStatus::SUCCESS;
  }

  void onFeedback(const FeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<FeedbackType>(*feedback));
  }

  BT::NodeStatus onAborted(const ResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "Object drag and drop failed");

    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(DragAndDrop);
};
}  // namespace thorp::bt::actions
