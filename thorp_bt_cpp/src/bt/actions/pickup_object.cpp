#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <thorp_msgs/PickupObjectAction.h>

namespace thorp::bt::actions
{
/**
 * Pickup a given object from a support surface.
 */
class PickupObject : public BT::RosActionNode<thorp_msgs::PickupObjectAction>
{
public:
  PickupObject(const std::string& name, const BT::NodeConfig& config) : RosActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("manipulation/pickup_object");
    ports.insert({ BT::InputPort<std::string>("object_name"),   //
                   BT::InputPort<std::string>("support_surf"),  //
                   BT::InputPort<float>("max_effort"),          //
                   BT::InputPort<float>("tightening"),          //
                   BT::OutputPort<int>("error"),                //
                   BT::OutputPort<std::optional<FeedbackType>>("feedback") });
    return ports;
  }

private:
  GoalType getGoal() override
  {
    GoalType goal;
    goal.object_name = *getInput<std::string>("object_name");
    goal.support_surf = *getInput<std::string>("support_surf");
    goal.max_effort = *getInput<float>("max_effort");
    goal.tightening = *getInput<float>("tightening");
    return goal;
  }

  void onFeedback(const FeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<FeedbackType>(*feedback));
  }

  BT::NodeStatus onAborted(const ResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "Error %d: %s", res->error.code, res->error.text.c_str());
    setOutput<int>("error", res->error.code);

    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(PickupObject);
};
}  // namespace thorp::bt::actions
