/*
 * Author: Jorge Santos
 */

#include <ros/ros.h>

// interactive manipulation markers action server
#include <actionlib/server/simple_action_server.h>
#include <interactive_markers/interactive_marker_server.h>

#include <thorp_msgs/DragAndDropAction.h>

// auxiliary libraries
#include <thorp_toolkit/tf2.hpp>
#include <thorp_toolkit/math.hpp>
#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/planning_scene.hpp>
namespace ttk = thorp_toolkit;


using namespace visualization_msgs;


namespace thorp_manipulation
{

class InteractiveManipulationServer
{
private:
  // Thorp interactive manipulation action server; by now just drag & drop interaction
  actionlib::SimpleActionServer<thorp_msgs::DragAndDropAction> as_;

  // Interactive markers server and associated data
  interactive_markers::InteractiveMarkerServer im_;
  geometry_msgs::Pose old_pose_;

public:

  InteractiveManipulationServer(const std::string name) :
    im_("move_objects"),
    as_(name, false)
  {
    // Register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&InteractiveManipulationServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&InteractiveManipulationServer::preemptCB, this));
    as_.start();
  }

  void goalCB()
  {
    thorp_msgs::DragAndDropGoal::ConstPtr goal = as_.acceptNewGoal();

    ROS_INFO("[interactive manip] Received goal! Adding markers for objects in the word other than the table");
    addObjects(goal->object_names, goal->support_surf, goal->output_frame);
  }

  void preemptCB()
  {
    ROS_WARN("[interactive manip] Action preempted");

    // set the action state to preempted
    as_.setPreempted();
  }

  // Moving an object; keep MOUSE_DOWN pose (origin) and move the object to MOUSE_UP pose
  void feedbackCb(const InteractiveMarkerFeedbackConstPtr &feedback)
  {
    if (!as_.isActive())
    {
      ROS_INFO("[interactive manip] Got feedback but not active!");
      return;
    }
    switch (feedback->event_type)
    {
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_INFO_STREAM("[interactive manip] Staging '" << feedback->marker_name << "' at " << feedback->pose);
        old_pose_ = feedback->pose;
        break;

      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        ROS_INFO_STREAM("[interactive manip] Now moving '" << feedback->marker_name << "' to " << feedback->pose);
        moveObject(feedback->marker_name, feedback->header, old_pose_, feedback->pose);
        break;
    }

    im_.applyChanges();
  }

  void moveObject(const std::string& marker_name, const std_msgs::Header& poses_header,
                  const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    thorp_msgs::DragAndDropResult result;
    result.object_name = marker_name;
    result.pick_pose.header = poses_header;
    result.place_pose.header = poses_header;
    result.pick_pose.pose = start_pose;
    result.place_pose.pose = end_pose;
    result.place_pose.pose.position.z += 0.005;

    as_.setSucceeded(result);

    im_.clear();
    im_.applyChanges();
  }

  // Add an interactive marker for any object in the word other than the table
  void addObjects(const std::vector<std::string> &object_names,
                  const std::string &support_surf, const std::string &output_frame)
  {
    im_.clear();
    im_.applyChanges();

    bool active = as_.isActive();

    for (const std::string& obj_name: object_names)
    {
      geometry_msgs::PoseStamped obj_pose; geometry_msgs::Vector3 obj_size;
      if (thorp_toolkit::getObjectData(obj_name, obj_pose, obj_size) > 0)
      {
        ttk::TF2::transformPose(output_frame, obj_pose, obj_pose);
        addMarker(obj_name, obj_pose, obj_size, active);
      }
    }

    im_.applyChanges();
  }


  // Add an interactive marker for the given object
  bool addMarker(const std::string& obj_name,
                 const geometry_msgs::PoseStamped& obj_pose, const geometry_msgs::Vector3& obj_size, bool active)
  {
    InteractiveMarker marker;
    marker.name = obj_name;
    marker.pose = obj_pose.pose;
    marker.header = obj_pose.header;
    marker.header.stamp = ros::Time();  // use object frame

    // We use the biggest dimension of the mesh to scale the marker
    marker.scale = ttk::maxValue(obj_size);

    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

    if (active)
      marker.controls.push_back(control);

    control.markers.push_back(makeBox(marker, 0.5, 0.5, 0.5));
    control.markers.push_back(makeLabel(marker, 0.5, 0.5, 0.5));
    control.always_visible = true;
    marker.controls.push_back(control);

    im_.insert(marker);
    im_.setCallback(marker.name, boost::bind(&InteractiveManipulationServer::feedbackCb, this, _1));

    ROS_INFO("[interactive manip] Added interactive marker for object '%s' at [%s] and scale [%f]",
             marker.name.c_str(), ttk::pose2cstr3D(marker.pose), marker.scale);

    return true;
  }

  // Make a box containing the object (5% bigger than the biggest dimension)
  Marker makeBox(InteractiveMarker &msg, float r, float g, float b)
  {
    Marker m;

    m.type = Marker::CUBE;
    m.scale.x = msg.scale + (msg.scale * 0.05);
    m.scale.y = msg.scale + (msg.scale * 0.05);
    m.scale.z = msg.scale + (msg.scale * 0.05);
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.1;

    return m;
  }

  // Make a label to show over the box
  Marker makeLabel(InteractiveMarker &msg, float r, float g, float b)
  {
    Marker m;

    m.type = Marker::TEXT_VIEW_FACING;
    m.text = msg.name;
    m.scale.x = 0.035;
    m.scale.y = 0.035;
    m.scale.z = 0.035;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.8;

    m.pose.position.z = msg.scale/2.0 + 0.025;

    return m;
  }

};

};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "object_interactive_manip_action_server");

  thorp_manipulation::InteractiveManipulationServer manip("drag_and_drop");

  ros::spin();

  return 0;
}
