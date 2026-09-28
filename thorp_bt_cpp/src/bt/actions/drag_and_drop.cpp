#include <cmath>
#include <map>
#include <mutex>
#include <optional>

#include <behaviortree_cpp/action_node.h>
#include <interactive_markers/interactive_marker_server.hpp>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>

namespace thorp::bt::actions
{
/**
 * Wait for the user to drag and drop a tabletop object on RViz, through the move_objects interactive markers, and
 * return its name, and its pickup and place poses. The place pose is the gripper's, as the place action takes it:
 * the object's top once resting at the drop position, placing_height_on_table above.
 */
class DragAndDrop : public BT::StatefulActionNode
{
public:
  DragAndDrop(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), markers_("move_objects", rosNode(*this))
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<moveit_msgs::msg::CollisionObject>>("objects"),  //
             BT::OutputPort<std::string>("object_name"),                                //
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("pickup_pose"),            //
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("place_pose") };
  }

private:
  struct Drop
  {
    std::string object_name;
    geometry_msgs::msg::PoseStamped pickup_pose;
    geometry_msgs::msg::PoseStamped place_pose;
  };

  interactive_markers::InteractiveMarkerServer markers_;
  // bt_server receives the markers feedback on another thread than the one ticking the tree; the markers server calls
  // us holding its own lock, so we never call it holding ours
  std::mutex mutex_;
  std::optional<Drop> drop_;
  std::map<std::string, double> heights_;
  geometry_msgs::msg::Pose drag_start_;

  BT::NodeStatus onStart() override
  {
    std::vector<std::string> ids;
    for (const auto& object : requireInput<std::vector<moveit_msgs::msg::CollisionObject>>(*this, "objects"))
      ids.push_back(object.id);

    const auto objects = planningScene().getObjects(ids);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      drop_.reset();
      heights_.clear();
      for (const auto& [id, object] : objects)
        heights_[id] = objectHeight(object);
    }
    markers_.clear();
    for (const auto& [id, object] : objects)
      markers_.insert(makeMarker(object), [this](const auto& feedback) { feedbackCB(feedback); });
    markers_.applyChanges();
    RCLCPP_INFO(logger(*this), "Drag and drop an object: %zu available", objects.size());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::optional<Drop> drop;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      drop = drop_;
    }
    if (!drop)
      return BT::NodeStatus::RUNNING;

    RCLCPP_INFO(logger(*this), "Drag and drop %s", drop->object_name.c_str());
    setOutput("object_name", drop->object_name);
    setOutput("pickup_pose", drop->pickup_pose);
    setOutput("place_pose", drop->place_pose);
    markers_.clear();
    markers_.applyChanges();
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
    markers_.clear();
    markers_.applyChanges();
  }

  /**
   * Keep the pose where the user grabs an object, and on release, take it and the release pose as a drop.
   */
  void feedbackCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
  {
    using visualization_msgs::msg::InteractiveMarkerFeedback;
    std::lock_guard<std::mutex> lock(mutex_);
    if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_DOWN)
    {
      drag_start_ = feedback->pose;
    }
    else if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP && !drop_)
    {
      Drop drop;
      drop.object_name = feedback->marker_name;
      drop.pickup_pose.header = drop.place_pose.header = feedback->header;
      drop.pickup_pose.pose = drag_start_;
      drop.place_pose.pose = feedback->pose;
      drop.place_pose.pose.position.z +=
          heights_[feedback->marker_name] / 2.0 + rosNode(*this)->get_parameter_or("placing_height_on_table", 0.005);
      drop_ = drop;
    }
  }

  /**
   * A box around the object, labeled with its name, that the user can move on the horizontal plane.
   */
  static visualization_msgs::msg::InteractiveMarker makeMarker(const moveit_msgs::msg::CollisionObject& object)
  {
    using visualization_msgs::msg::InteractiveMarkerControl;
    using visualization_msgs::msg::Marker;
    const Eigen::Vector3d size =
        object.meshes.empty() ? Eigen::Vector3d(0.03, 0.03, 0.03)
                              : shapes::computeShapeExtents(shapes::ShapeMsg(object.meshes.front()));

    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = object.header.frame_id;
    marker.name = object.id;
    marker.pose = object.pose;
    marker.scale = static_cast<float>(size.maxCoeff());

    Marker box;
    box.type = Marker::CUBE;
    box.scale.x = box.scale.y = box.scale.z = marker.scale * 1.05;
    box.color.r = box.color.g = box.color.b = 0.5;
    box.color.a = 0.1;

    Marker label;
    label.type = Marker::TEXT_VIEW_FACING;
    label.text = object.id;
    label.scale.x = label.scale.y = label.scale.z = 0.035;
    label.color.r = label.color.g = label.color.b = 0.5;
    label.color.a = 0.8;
    label.pose.position.z = marker.scale / 2.0 + 0.025;
    label.pose.orientation.w = 1.0;

    // Moving on the plane normal to the control's x axis, turned vertical
    InteractiveMarkerControl control;
    control.orientation.w = control.orientation.y = std::sqrt(2.0) / 2.0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    control.always_visible = true;
    control.markers = { box, label };
    marker.controls.push_back(control);
    return marker;
  }

  BT_REGISTER_NODE(DragAndDrop);
};
}  // namespace thorp::bt::actions
