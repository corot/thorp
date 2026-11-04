#include <optional>

#include "thorp_bt_cpp/ros_action_node.hpp"

#include "thorp_bt_cpp/node_common.hpp"

#include <thorp_msgs/action/pickup_object.hpp>

namespace thorp::bt::actions
{
/**
 * Pickup a given object from a support surface.
 */
class PickupObject : public BT::RosActionNode<thorp_msgs::action::PickupObject>
{
public:

  PickupObject(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : RosActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::string>("object_name"),             //
                                BT::InputPort<std::string>("support_surf"),            //
                                BT::InputPort<float>("max_effort"),                    //
                                BT::InputPort<float>("tightening"),                    //
                                BT::OutputPort<thorp_msgs::msg::ThorpError>("error"),  //
                                BT::OutputPort<std::optional<Feedback>>("feedback") });
  }

private:
  Goal getGoal() override
  {
    Goal goal;
    goal.object_name = requireInput<std::string>(*this, "object_name");
    goal.support_surf = requireInput<std::string>(*this, "support_surf");
    goal.max_effort = requireInput<float>(*this, "max_effort");
    goal.tightening = requireInput<float>(*this, "tightening");
    return goal;
  }

  void onFeedback(const Feedback& feedback) override
  {
    setOutput("feedback", std::make_optional<Feedback>(feedback));
  }

  BT::NodeStatus onAborted(const Result& result) override
  {
    RCLCPP_ERROR(logger(*this), "Error %d: %s", result.error.code, result.error.text.c_str());
    setOutput("error", result.error);
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_ACTION_NODE(PickupObject, "manipulation/pickup_object");
};

}  // namespace thorp::bt::actions
