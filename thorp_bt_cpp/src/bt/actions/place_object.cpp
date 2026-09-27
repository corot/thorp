#include <optional>

#include <nav2_behavior_tree/bt_action_node.hpp>

#include "thorp_bt_cpp/node_common.hpp"

#include <thorp_msgs/action/place_object.hpp>

namespace thorp::bt::actions
{
/**
 * Place a given object on a support surface.
 */
class PlaceObject : public nav2_behavior_tree::BtActionNode<thorp_msgs::action::PlaceObject>
{
public:
  using ActionType = thorp_msgs::action::PlaceObject;

  PlaceObject(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : BtActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::string>("object_name"),                     //
                                BT::InputPort<std::string>("support_surf"),                    //
                                BT::InputPort<geometry_msgs::msg::PoseStamped>("place_pose"),  //
                                BT::OutputPort<int>("error"),                                  //
                                BT::OutputPort<std::optional<ActionType::Feedback>>("feedback") });
  }

private:
  void on_tick() override
  {
    goal_.object_name = requireInput<std::string>(*this, "object_name");
    goal_.support_surf = requireInput<std::string>(*this, "support_surf");
    goal_.place_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "place_pose");
  }

  void on_wait_for_result(std::shared_ptr<const ActionType::Feedback> feedback) override
  {
    if (feedback)
      setOutput("feedback", std::make_optional<ActionType::Feedback>(*feedback));
  }

  BT::NodeStatus on_aborted() override
  {
    RCLCPP_ERROR(logger(*this), "Error %d: %s", result_.result->error.code, result_.result->error.text.c_str());
    setOutput<int>("error", result_.result->error.code);
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_ACTION_NODE(PlaceObject, "manipulation/place_object");
};

}  // namespace thorp::bt::actions
