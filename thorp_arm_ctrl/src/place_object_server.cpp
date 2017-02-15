/*
 * Author: Jorge Santos
 */

#include <tf/tf.h>
#include <ros/ros.h>

// auxiliary libraries
#include <yocs_math_toolkit/common.hpp>

// MoveIt!
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <moveit_msgs/PlaceLocation.h>

// Thorp stuff
#include <thorp_toolkit/planning_scene.hpp>

#include "thorp_arm_ctrl/place_object_server.hpp"


namespace thorp_arm_ctrl
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
  ROS_INFO("[place object] Execute goal: place object '%s' on support surface '%s' at pose [%s]...",
           goal->object_name.c_str(), goal->support_surf.c_str(), mtk::pose2str3D(goal->place_pose).c_str());

  thorp_msgs::PlaceObjectResult result;
  result.error.code = place(goal->object_name, goal->support_surf, goal->place_pose);
  result.error.text = mec2str(result.error.code);
  if (result.error.code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    as_.setSucceeded(result);
  }
  else if (result.error.code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    as_.setPreempted(result);
  }
  else
  {
    as_.setAborted(result);

    // Ensure we don't retain any object attached to the gripper
//    arm().detachObject(goal->object_name);
//    setGripper(gripper_open, false);
  }
}

void PlaceObjectServer::preemptCB()
{
  ROS_WARN("[place object] Action preempted; cancel all movement");
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

  ROS_INFO("[place object] Placing object '%s' at pose [%s]...", obj_name.c_str(), mtk::pose2str3D(pose).c_str());

  moveit_msgs::PlaceGoal goal;
  goal.attached_object_name = obj_name;
  goal.group_name = arm().getName();
  goal.allowed_planning_time = arm().getPlanningTime();
  goal.support_surface_name = surface;
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

  ac_.sendGoal(goal);

  while (!ac_.waitForResult(ros::Duration(0.1)))
  {
    if (as_.isPreemptRequested())
    {
      ROS_WARN("[place object] preempt.................................");
      ac_.cancelAllGoals();
      ROS_WARN("[place object] Place action preempted    %d", ac_.getResult()->error_code.val);
      ///return ac_.getResult()->error_code.val;
      return moveit::planning_interface::MoveItErrorCode::PREEMPTED;
    }
  }

  ROS_WARN("[place object] DONE");
  if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("[place object] Place succeeded!");
  }
  else
  {
    ROS_ERROR("[place object] Place fail with error code [%d] (%s): %s",
              ac_.getResult()->error_code.val, ac_.getState().toString().c_str(), ac_.getState().getText().c_str());
  }

  return ac_.getResult()->error_code.val;
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
    if (!validateTargetPose(p, true, false, false, attempt))
    {
      return thorp_msgs::ThorpError::INVALID_TARGET_POSE;
    }

    ROS_DEBUG("[place object] Place attempt %d at pose [%s]...", attempt, mtk::pose2str3D(p).c_str());

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
              mtk::pose2str3D(attached_obj_pose).c_str(), mtk::pose2str3D(p.pose).c_str());

    moveit_msgs::PlaceLocation l;
    l.place_pose = p;

    l.pre_place_approach.direction.vector.x = 0.5;
    l.pre_place_approach.direction.header.frame_id = arm().getEndEffectorLink();
    l.pre_place_approach.min_distance = 0.005;
    l.pre_place_approach.desired_distance = 0.1;

    l.post_place_retreat.direction.vector.x = -0.5;
    l.post_place_retreat.direction.header.frame_id = arm().getEndEffectorLink();
    l.post_place_retreat.min_distance = 0.005;
    l.post_place_retreat.desired_distance = 0.1;

    l.post_place_posture.joint_names.push_back("gripper_joint");
    l.post_place_posture.points.resize(1);
    l.post_place_posture.points[0].positions.push_back(gripper_open);

    l.allowed_touch_objects.push_back(obj_name);
    l.allowed_touch_objects.push_back(surface);

    l.id = attempt;

    place_locations.push_back(l);
  }

  return place_locations.size();
}

};
