/*
 * Author: Jorge Santos
 */

#include <tf/tf.h>
#include <ros/ros.h>

// auxiliary libraries
#include <yocs_math_toolkit/common.hpp>

// MoveIt!
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <moveit_msgs/Grasp.h>

// Thorp stuff
#include <thorp_toolkit/planning_scene.hpp>

#include "thorp_arm_ctrl/pickup_object_server.hpp"


namespace thorp_arm_ctrl
{

PickupObjectServer::PickupObjectServer(const std::string name) :
  action_name_(name),
  as_(name, boost::bind(&PickupObjectServer::executeCB, this, _1), false),
  ac_(move_group::PICKUP_ACTION, true)
{
  // Wait for MoveIt! pickup action server
  ROS_INFO("[pickup object] Waiting for MoveIt! pickup action server...");
  ac_.waitForServer();
  ROS_INFO("[pickup object] Available! Starting our own server...");

  // Register feedback callback for our server; executeCB is run on a separated thread, so it can be cancelled
  as_.registerPreemptCallback(boost::bind(&PickupObjectServer::preemptCB, this));
  as_.start();
}

PickupObjectServer::~PickupObjectServer()
{
  as_.shutdown();
}

void PickupObjectServer::executeCB(const thorp_msgs::PickupObjectGoal::ConstPtr& goal)
{
  ROS_INFO("[pickup object] Execute goal: pick object '%s' from support surface '%s'",
           goal->object_name.c_str(), goal->support_surf.c_str());

  thorp_msgs::PickupObjectResult result;
  result.error.code = pickup(goal->object_name, goal->support_surf);
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
 //   arm().detachObject(goal->object_name);
   // setGripper(gripper_open, false);

  }
}

void PickupObjectServer::preemptCB()
{
  ROS_WARN("[pickup object] Action preempted; cancel all movement");
  gripper().stop();
  arm().stop();
}

int32_t PickupObjectServer::pickup(const std::string& obj_name, const std::string& surface)
{
  if (!ac_.isServerConnected())
  {
    ROS_ERROR("[pickup object] MoveIt! pickup action server not connected");
    return thorp_msgs::ThorpError::SERVER_NOT_AVAILABLE;
  }

  // Look for obj_name in the planning scene's list of collision objects
  geometry_msgs::PoseStamped obj_pose; geometry_msgs::Vector3 obj_size;
  int32_t result = thorp_toolkit::getObjectData(obj_name, obj_pose, obj_size);
  if (result < 0)
  {
    // Error occurred while getting object data...
    return result;
  }

  ROS_INFO("[pickup object] Picking object '%s' with size [%s] at location [%s]...",
           obj_name.c_str(), mtk::vector2str3D(obj_size).c_str(), mtk::point2str2D(obj_pose.pose.position).c_str());

  // Prepare and send pick goal
  moveit_msgs::PickupGoal goal;
  goal.target_name = obj_name;
  goal.group_name = arm().getName();
  goal.end_effector = gripper().getName();
  goal.support_surface_name = surface;
  goal.allowed_planning_time = arm().getPlanningTime();
  goal.allow_gripper_support_collision = true;
  goal.planning_options.plan_only = false;
  goal.planning_options.look_around = false;
  goal.planning_options.replan = true;
  goal.planning_options.replan_delay = 0.1; // ???
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  result = makeGrasps(obj_pose, obj_size, obj_name, surface, goal.possible_grasps);
  if (result < 0)
  {
    // Error occurred while making grasps...
    return result;
  }


  //string[] attached_object_touch_links --> defaults to gripper; can change by just fingers


//  # Optionally notify the pick action that it should approach the object further,
//  # as much as possible (this minimizing the distance to the object before the grasp)
//  # along the approach direction; Note: this option changes the grasping poses
//  # supplied in possible_grasps[] such that they are closer to the object when possible.
//  goal.minimize_object_distance  -->  TRY

//  # an optional list of obstacles that we have semantic information about
//  # and that can be touched/pushed/moved in the course of grasping;
//  # CAREFUL: If the object name 'all' is used, collisions with all objects are disabled during the approach & lift.
//  string[] allowed_touch_objects

//  # The maximum amount of time the motion planner is allowed to plan for
//  float64 allowed_planning_time
//
//  # Planning options
//  PlanningOptions planning_options

//  arm().setSupportSurfaceName(goal->support_surf);
//
//  // Allow some leeway in position (meters) and orientation (radians)
//  arm().setGoalPositionTolerance(0.001);
//  arm().setGoalOrientationTolerance(0.02);
//
//  // Allow replanning to increase the odds of a solution
//  arm().allowReplanning(true);

  ac_.sendGoal(goal);

  while (!ac_.waitForResult(ros::Duration(0.1)))
  {
    ROS_WARN("[pickup object] NOT");
    if (as_.isPreemptRequested())
    {
      ROS_WARN("[pickup object] preempt.................................");
      ac_.cancelAllGoals();
      ROS_WARN("[pickup object] Pick action preempted    %d", ac_.getResult()->error_code.val);
      ///return ac_.getResult()->error_code.val;
      return moveit::planning_interface::MoveItErrorCode::PREEMPTED;
    }
  }

  result = ac_.getResult()->error_code.val;
  ROS_WARN("[pickup object] DONE");
  if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("[pickup object] Pick succeeded!");
  }
  else
  {
    ROS_ERROR("[pickup object] Pick fail with error code [%d] (%s): %s",
              ac_.getResult()->error_code.val, ac_.getState().toString().c_str(), ac_.getState().getText().c_str());
  }

  ROS_WARN("[pickup object] Pick OOOOOOOOOOOOK>>>>>>>>>>>>>    %d", ac_.getResult()->error_code.val);
  return ac_.getResult()->error_code.val;
//  {
//    return moveit::planning_interface::MoveItErrorCode(ac_.getResult()->error_code);
//  }
//  else
//  {
//    return moveit::planning_interface::MoveItErrorCode(ac_.getResult()->error_code);
//  }
//
//    if ((result = arm().pick(obj_name, grasps)))
//    {
//      ROS_INFO("[pickup object] Pick successfully completed");
//      return result.val;
//    }
//
//    ROS_DEBUG("[pickup object] Pick attempt %d failed: %s", attempt, mec2str(result));
//
//
//  ROS_ERROR("[pickup object] Pick failed after %d attempts", PICK_ATTEMPTS);
//  return result.val;
}


int32_t PickupObjectServer::makeGrasps(const geometry_msgs::PoseStamped& target_pose,
                                       const geometry_msgs::Vector3& target_size,
                                       const std::string& obj_name, const std::string& surface,
                                       std::vector<moveit_msgs::Grasp>& grasps)
{
  // Try up to PICK_ATTEMPTS grasps with slightly different poses

  for (int attempt = 0; attempt < PICK_ATTEMPTS; ++attempt)
  {
    geometry_msgs::PoseStamped p = target_pose;
    if (!validateTargetPose(p, true, true, true, attempt))
    {
      return thorp_msgs::ThorpError::INVALID_TARGET_POSE;
    }

    ROS_DEBUG("[pickup object] Pick attempt %d at pose [%s]...", attempt, mtk::pose2str3D(p).c_str());

    moveit_msgs::Grasp g;
    g.grasp_pose = p;

    g.pre_grasp_approach.direction.vector.x = 0.5;
    g.pre_grasp_approach.direction.header.frame_id = arm().getEndEffectorLink();
    g.pre_grasp_approach.min_distance = 0.005;
    g.pre_grasp_approach.desired_distance = 0.1;

    g.post_grasp_retreat.direction.header.frame_id = arm().getEndEffectorLink();
    g.post_grasp_retreat.direction.vector.x = -0.5;
    g.post_grasp_retreat.min_distance = 0.005;
    g.post_grasp_retreat.desired_distance = 0.1;

    g.pre_grasp_posture.joint_names.push_back("gripper_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.push_back(gripper_open);

    // As we grasp the object "blindly", just in the center, we use the maximum possible value as the opened
    // gripper position and the smallest dimension minus a small "tightening" epsilon as the closed position
    g.grasp_posture.joint_names.push_back("gripper_joint");
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.push_back(std::min(target_size.x, target_size.y) - 0.002);

    g.allowed_touch_objects.push_back(obj_name);
    g.allowed_touch_objects.push_back(surface);

    g.id = attempt;

    grasps.push_back(g);
  }

  return grasps.size();
}

};
