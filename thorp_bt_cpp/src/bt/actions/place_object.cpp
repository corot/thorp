#include <optional>

#include "thorp_bt_cpp/ros_action_node.hpp"

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"

#include <thorp_msgs/action/place_object.hpp>
#include <thorp_toolkit/tray.hpp>

namespace thorp::bt::actions
{
/**
 * Place a given object on a support surface.
 */
class PlaceObject : public BT::RosActionNode<thorp_msgs::action::PlaceObject>
{
public:

  PlaceObject(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : RosActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::string>("object_name"),                     //
                                BT::InputPort<std::string>("support_surf"),                    //
                                BT::InputPort<geometry_msgs::msg::PoseStamped>("place_pose"),  //
                                BT::OutputPort<thorp_msgs::msg::ThorpError>("error"),          //
                                BT::OutputPort<std::optional<Feedback>>("feedback") });
  }

private:
  Goal getGoal() override
  {
    Goal goal;
    goal.object_name = requireInput<std::string>(*this, "object_name");
    goal.support_surf = requireInput<std::string>(*this, "support_surf");
    goal.place_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "place_pose");
    // Tray slots are tight, so placing on it can touch the objects already there
    goal.allowed_touch_objects.clear();
    const thorp::toolkit::Tray tray;
    if (goal.support_surf == tray.link())
    {
      std::vector<moveit_msgs::msg::CollisionObject> on_tray;
      for (const auto& [name, object] : planningScene().getObjects())
      {
        if (!tray.onTray(object))
          continue;
        on_tray.push_back(object);
        goal.allowed_touch_objects.push_back(name);
      }
      RCLCPP_INFO_EXPRESSION(logger(*this), !on_tray.empty(), "Objects on the tray we can touch: %s",
                             objectIds(on_tray).c_str());
    }
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

  BT_REGISTER_ACTION_NODE(PlaceObject, "manipulation/place_object");
};

}  // namespace thorp::bt::actions
