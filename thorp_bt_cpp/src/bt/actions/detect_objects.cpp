#include <unordered_set>

#include <nav2_behavior_tree/bt_action_node.hpp>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <thorp_msgs/action/detect_objects.hpp>

#include <thorp_toolkit/common.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Detect the objects on the table in front of the robot, keeping only those of the given types, if any.
 */
class DetectObjects : public nav2_behavior_tree::BtActionNode<thorp_msgs::action::DetectObjects>
{
public:
  DetectObjects(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : BtActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::string>("object_types"),                                //
                                BT::OutputPort<std::vector<moveit_msgs::msg::CollisionObject>>("objects"),  //
                                BT::OutputPort<moveit_msgs::msg::CollisionObject>("surface") });
  }

private:
  void on_tick() override
  {
    goal_.clear_scene = false;
  }

  BT::NodeStatus on_success() override
  {
    std::unordered_set<std::string> valid_targets;
    auto object_types_csv = requireInput<std::string>(*this, "object_types");
    if (!object_types_csv.empty())
    {
      auto object_types = ttk::tokenize(object_types_csv);
      std::copy(object_types.begin(), object_types.end(), inserter(valid_targets, valid_targets.begin()));
      RCLCPP_INFO_STREAM(logger(*this), "Detecting " << object_types_csv);
    }
    else
    {
      RCLCPP_INFO_STREAM(logger(*this), "No object types provided; reporting all detections");
    }

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    for (const auto& object : result_.result->objects)
    {
      const auto obj_type = object.id.substr(0, object.id.find(' '));  // remove index
      if (valid_targets.empty() || valid_targets.find(obj_type) != valid_targets.end())
      {
        objects.emplace_back(object);
      }
    }

    RCLCPP_INFO_STREAM(logger(*this), objects.size() << " object(s) detected: " << objectIds(objects));
    setOutput("objects", objects);
    setOutput("surface", result_.result->surface);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus on_aborted() override
  {
    RCLCPP_ERROR(logger(*this), "Object detection failed");
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_ACTION_NODE(DetectObjects, "object_detection");
};

}  // namespace thorp::bt::actions
