/*
 * Author: Jorge Santos
 */

#include <tf/tf.h>
#include <ros/ros.h>

// auxiliary libraries
#include <thorp_toolkit/tf.hpp>
namespace ttk = thorp_toolkit;

// MoveIt!
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <moveit_msgs/PlaceLocation.h>

// Thorp stuff
#include <thorp_msgs/ThorpError.h>
#include <thorp_toolkit/planning_scene.hpp>

#include "thorp_manipulation/place_object_server.hpp"


namespace thorp_manipulation
{

PlaceObjectServer::PlaceObjectServer(const std::string name) :
  action_name_(name),
  as_(name, boost::bind(&PlaceObjectServer::executeCB, this, _1), false),
  ac_(move_group::PLACE_ACTION, true)
{
  // Wait for MoveIt! place action server
  ROS_INFO("[place object] Waiting for MoveIt! place action server...");
  ac_.waitForServer();
  ROS_INFO("[place object] Available! Starting our own server...");

  // Register feedback callback for our server; executeCB is run on a separated thread, so it can be cancelled
  as_.registerPreemptCallback(boost::bind(&PlaceObjectServer::preemptCB, this));
  as_.start();
}

PlaceObjectServer::~PlaceObjectServer()
{
  as_.shutdown();
}

void PlaceObjectServer::executeCB(const thorp_msgs::PlaceObjectGoal::ConstPtr& goal)
{
  preempted_ = false;
  thorp_msgs::PlaceObjectResult result;

  if (!goal->object_name.empty() && goal->object_name != attached_object)
  {
    ROS_ERROR("[place object] Requested object not in gripper: '%s'; currently attached object is '%s'",
              goal->object_name.c_str(), attached_object.c_str());
    result.error.code = thorp_msgs::ThorpError::OBJECT_NOT_ATTACHED;
    result.error.text = mec2str(result.error.code);
    as_.setAborted(result);
    return;
  }

  ROS_INFO("[place object] Execute goal: place object '%s' on support surface '%s' at pose [%s]...",
           goal->object_name.c_str(), goal->support_surf.c_str(), ttk::pose2cstr3D(goal->place_pose));

  result.error.code = place(goal->object_name, goal->support_surf, goal->place_pose);
  result.error.text = mec2str(result.error.code);
  if (result.error.code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    as_.setSucceeded(result);
    attached_object = "";
  }
  else if (result.error.code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    as_.setPreempted(result);
  }
  else
  {
    as_.setAborted(result);
  }
}

void PlaceObjectServer::preemptCB()
{
  ROS_WARN("[place object] Action preempted; cancel all movement");
  preempted_ = true;
  gripper().stop();
  arm().stop();
}

int32_t PlaceObjectServer::place(const std::string& obj_name, const std::string& surface,
                                 const geometry_msgs::PoseStamped& pose)
{
  if (!ac_.isServerConnected())
  {
    ROS_ERROR("[place object] MoveIt! place action server not connected");
    return thorp_msgs::ThorpError::SERVER_NOT_AVAILABLE;
  }

  // Look for obj_name in the planning scene's list of attached collision objects
  geometry_msgs::Pose attached_pose;
  int32_t result = thorp_toolkit::getAttachedObjectPose(obj_name, attached_pose);
  if (result < 0)
  {
    // Error occurred while getting object data...
    return result;
  }

  ROS_INFO("[place object] Placing object '%s' at pose [%s]...", obj_name.c_str(), ttk::pose2cstr3D(pose));

  moveit_msgs::PlaceGoal goal;
  goal.attached_object_name = obj_name;
  goal.group_name = arm().getName();
  goal.support_surface_name = surface;
  goal.allowed_planning_time = arm().getPlanningTime();
  goal.allow_gripper_support_collision = true;
  goal.planning_options.plan_only = false;
  goal.planning_options.look_around = false;
  goal.planning_options.replan = true;
  goal.planning_options.replan_delay = 0.1;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  result = makePlaceLocations(pose, attached_pose, obj_name, surface, goal.place_locations);
  if (result < 0)
  {
    // Error occurred while making grasps...
    return result;
  }

  // Allow some leeway in position (meters) and orientation (radians)
  arm().setGoalPositionTolerance(0.001);  // TODO: same values already set on parent class; add to the goal if needed
  arm().setGoalOrientationTolerance(0.02);

  // Allow replanning to increase the odds of a solution
  arm().allowReplanning(true);

  ac_.sendGoal(goal);

  if (!ac_.waitForResult(ros::Duration(30)))  // TODO param or add to goal
  {
    preemptCB();
    ac_.cancelAllGoals();
    ROS_WARN("[place object] Place action timed out after 30s");
    return moveit::planning_interface::MoveItErrorCode::TIMED_OUT;
  }

  if (preempted_)
  {
    ac_.cancelAllGoals();
    ROS_WARN("[place object] Place action preempted");
    return moveit::planning_interface::MoveItErrorCode::PREEMPTED;
  }

  result = ac_.getResult()->error_code.val;
  if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("[place object] Place succeeded!");
  }
  else
  {
    ROS_ERROR("[place object] Place fail with error code [%d] (%s): %s",
              result, ac_.getState().toString().c_str(), ac_.getState().getText().c_str());
  }
  return result;
}

int32_t PlaceObjectServer::makePlaceLocations(const geometry_msgs::PoseStamped& target_pose,
                                              const geometry_msgs::Pose& attached_obj_pose,
                                              const std::string& obj_name, const std::string& surface,
                                              std::vector<moveit_msgs::PlaceLocation>& place_locations)
{
  // Try up to PLACE_ATTEMPTS place locations with slightly different poses

  for (int attempt = 0; attempt < PLACE_ATTEMPTS; ++attempt)
  {
    geometry_msgs::PoseStamped p = target_pose;
    if (!validateTargetPose(p, true, true, true, attempt))
    {
      return thorp_msgs::ThorpError::INVALID_TARGET_POSE;
    }

    ROS_DEBUG("[place object] Place attempt %d at pose [%s]...", attempt, ttk::pose2cstr3D(p));

    // MoveGroup::place will transform the provided place pose with the attached body pose, so the object retains
    // the orientation it had when picked. However, with our 4-dofs arm this is infeasible (nor we care about the
    // objects orientation!), so we cancel this transformation. It is applied here:
    // https://github.com/ros-planning/moveit_ros/blob/jade-devel/manipulation/pick_place/src/place.cpp#L64
    // More details on this issue: https://github.com/ros-planning/moveit_ros/issues/577
    tf::Transform place_tf, aco_tf;
    tf::poseMsgToTF(p.pose, place_tf);
    tf::poseMsgToTF(attached_obj_pose, aco_tf);
    tf::poseTFToMsg(place_tf * aco_tf, p.pose);

    ROS_DEBUG("[place object] Compensate place pose with the attached object pose [%s]. Results: [%s]",
              ttk::pose2cstr3D(attached_obj_pose), ttk::pose2cstr3D(p.pose));

    moveit_msgs::PlaceLocation l;
    l.place_pose = p;

    l.pre_place_approach.direction.vector.x = 0.5;
    l.pre_place_approach.direction.header.frame_id = arm().getEndEffectorLink();
    l.pre_place_approach.min_distance = 0.05;
    l.pre_place_approach.desired_distance = 0.1;

    l.post_place_retreat.direction.vector.x = -0.5;
    l.post_place_retreat.direction.header.frame_id = arm().getEndEffectorLink();
    l.post_place_retreat.min_distance = 0.05;
    l.post_place_retreat.desired_distance = 0.1;

    l.post_place_posture.joint_names.push_back("gripper_joint");
    l.post_place_posture.points.resize(1);
    l.post_place_posture.points[0].positions.push_back(gripper_open);

    l.allowed_touch_objects.push_back(obj_name);
    l.allowed_touch_objects.push_back(surface);

    l.id = std::to_string(attempt);

    place_locations.push_back(l);
  }

  return place_locations.size();
}

};
