#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <moveit_msgs/CollisionObject.h>
#include <thorp_msgs/DetectObjectsAction.h>

#include <thorp_toolkit/common.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * @brief Tabletop object detection.
 * Tries to segment a support surface and classify the tabletop objects found on it. Returns only the
 * objects of types listed on 'object_types' input key (or all if it's not provided).
 * As output, it returns the objects as a list of moveit_msgs/CollisionObject msgs and a single msg for
 * the support surface.
 * All detected objects and the support surface will be added to the planning scene as collision objects.
 * If clear_scene is true, the planning scene will be previously emptied.
 */
class DetectObjects : public BT::RosActionNode<thorp_msgs::DetectObjectsAction>
{
public:
  DetectObjects(const std::string& name, const BT::NodeConfiguration& config) : RosActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("object_detection");
    ports.insert({ BT::InputPort<std::string>("object_types"),                            //
                   BT::OutputPort<std::vector<moveit_msgs::CollisionObject>>("objects"),  //
                   BT::OutputPort<moveit_msgs::CollisionObject>("surface") });
    return ports;
  }

private:
  std::optional<GoalType> getGoal() override
  {
    if (status() == BT::NodeStatus::RUNNING)
      return std::nullopt;

    GoalType goal;
    goal.clear_scene = false;
    // TODO goal.output_frame = *getInput<std::string>("support_surf");
    return goal;
  }

  void onFeedback(const FeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<FeedbackType>(*feedback));
  }

  BT::NodeStatus onSucceeded(const ResultConstPtr& res) override
  {
    std::unordered_set<std::string> valid_targets;
    auto object_types_csv = *getInput<std::string>("object_types");
    if (!object_types_csv.empty())
    {
      auto object_types = ttk::tokenize(object_types_csv);
      std::copy(object_types.begin(), object_types.end(), inserter(valid_targets, valid_targets.begin()));
      ROS_INFO_STREAM_NAMED(name(), "Detecting " << object_types_csv);
    }
    else
    {
      ROS_INFO_STREAM_NAMED(name(), "No object types provided; reporting all detections");
    }

    std::vector<moveit_msgs::CollisionObject> objects;
    for (const auto& object : res->objects)
    {
      const auto obj_type = object.id.substr(0, object.id.find(' '));  // remove index
      if (valid_targets.empty() || valid_targets.find(obj_type) != valid_targets.end())
      {
        objects.emplace_back(object);
      }
    }
    ROS_INFO_NAMED(name(), "%lu objects detected", objects.size());
    setOutput("objects", objects);
    setOutput("surface", res->surface);

    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onAborted(const ResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "Object detection failed");

    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(DetectObjects);
};
}  // namespace thorp::bt::actions
