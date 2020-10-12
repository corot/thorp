/*
 * Author: Jorge Santos
 */

#include <ros/ros.h>

// auxiliary libraries
#include <thorp_toolkit/tf.hpp>
#include <thorp_toolkit/math.hpp>
namespace ttk = thorp_toolkit;

// MoveIt!
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <moveit_msgs/Grasp.h>

// Thorp stuff
#include <thorp_msgs/ThorpError.h>
#include <thorp_toolkit/planning_scene.hpp>

#include "thorp_manipulation/pickup_object_server.hpp"


namespace thorp_manipulation
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
  preempted_ = false;
  thorp_msgs::PickupObjectResult result;

  if (!attached_object.empty())
  {
    ROS_ERROR("[pickup object] Already have an attached object: '%s'; clear the gripper before picking another",
              attached_object.c_str());
    result.error.code = thorp_msgs::ThorpError::OBJECT_ATTACHED;
    result.error.text = mec2str(result.error.code);
    as_.setAborted(result);
    return;
  }

  ROS_INFO("[pickup object] Execute goal: pick object '%s' from support surface '%s' exerting up to %.2fN",
           goal->object_name.c_str(), goal->support_surf.c_str(), goal->max_effort);

  result.error.code = pickup(goal->object_name, goal->support_surf, goal->max_effort);
  result.error.text = mec2str(result.error.code);
  if (result.error.code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    as_.setSucceeded(result);
    attached_object = goal->object_name;
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

void PickupObjectServer::preemptCB()
{
  ROS_WARN("[pickup object] Action preempted; cancel all movement");
  preempted_ = true;
  gripper().stop();
  arm().stop();
}

int32_t PickupObjectServer::pickup(const std::string& obj_name, const std::string& surface, float max_effort)
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
           obj_name.c_str(), ttk::vector2cstr3D(obj_size), ttk::point2cstr2D(obj_pose.pose.position));

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

  result = makeGrasps(obj_pose, obj_size, obj_name, surface, max_effort, goal.possible_grasps);
  if (result < 0)
  {
    // Error occurred while making grasps...
    return result;
  }

  // TODO: play with the many other options:
  // goal.attached_object_touch_links : string[] --> defaults to gripper; can change by just fingers

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
    ROS_WARN("[pickup object] Pick action timed out after 30s");
    return moveit::planning_interface::MoveItErrorCode::TIMED_OUT;
  }

  if (preempted_)
  {
    ac_.cancelAllGoals();
    ROS_WARN("[pickup object] Pick action preempted");
    return moveit::planning_interface::MoveItErrorCode::PREEMPTED;
  }

  result = ac_.getResult()->error_code.val;
  if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("[pickup object] Pick succeeded!");
  }
  else
  {
    ROS_ERROR("[pickup object] Pick fail with error code [%d] (%s): %s",
              result, ac_.getState().toString().c_str(), ac_.getState().getText().c_str());
  }
  return result;
}


int32_t PickupObjectServer::makeGrasps(const geometry_msgs::PoseStamped& obj_pose,
                                       const geometry_msgs::Vector3& obj_size,
                                       const std::string& obj_name, const std::string& surface, float max_effort,
                                       std::vector<moveit_msgs::Grasp>& grasps)
{
  // Try up to PICK_ATTEMPTS grasps with slightly different poses

  for (int attempt = 0; attempt < PICK_ATTEMPTS; ++attempt)
  {
    geometry_msgs::PoseStamped target_pose = obj_pose;
    if (!validateTargetPose(target_pose, true, true, true, attempt))
    {
      return thorp_msgs::ThorpError::INVALID_TARGET_POSE;
    }

    ROS_DEBUG("[pickup object] Pick attempt %d at pose [%s]...", attempt, ttk::pose2cstr3D(target_pose));

    moveit_msgs::Grasp g;
    g.grasp_pose = target_pose;

    g.pre_grasp_approach.direction.vector.x = 0.5;
    g.pre_grasp_approach.direction.header.frame_id = arm().getEndEffectorLink();
    g.pre_grasp_approach.min_distance = 0.05;
    g.pre_grasp_approach.desired_distance = 0.1;

    g.post_grasp_retreat.direction.header.frame_id = arm().getEndEffectorLink();
    g.post_grasp_retreat.direction.vector.x = -0.5;
    g.post_grasp_retreat.min_distance = 0.05;
    g.post_grasp_retreat.desired_distance = 0.1;

    g.pre_grasp_posture.joint_names.push_back("gripper_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.push_back(gripper_open);

    // As we grasp the object "blindly", just in the center, we use the maximum possible value as the opened gripper
    // position and the dimension more aligned with the arm yaw, minus a "tightening" factor, as the closed position
    g.grasp_posture.joint_names.push_back("gripper_joint");
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.push_back(std::min(obj_size.x, obj_size.y) * 0.80);  // TODO: still use the smallest dimension minus a 20% "tightening"
    //g.grasp_posture.points[0].positions.push_back(gripperClosing(target_pose, obj_size));  // TODO: useless until my obj rec reports orientation!
    g.grasp_posture.points[0].effort.push_back(max_effort);

    g.allowed_touch_objects.push_back(obj_name);
    g.allowed_touch_objects.push_back(surface);

    g.id = std::to_string(attempt);

    grasps.push_back(g);
  }

  return grasps.size();
}

double PickupObjectServer::gripperClosing(const geometry_msgs::PoseStamped& target_pose,
                                          const geometry_msgs::Vector3& target_size)
{
  double h = ttk::heading(target_pose.pose);  // assuming is in arm reference frame
  double y = ttk::yaw(target_pose.pose);
  double s = std::abs(ttk::wrapAngle(y - h)) < M_PI/2.0 ? target_size.y : target_size.x;

  ROS_WARN_STREAM(" GRIPPER CLOSING  "<< h << "  "<< y << "   " << (std::abs(ttk::wrapAngle(y - h)) < M_PI/2.0 ? " USE Y    " : "  USE X    ")
  << target_size.x << "  "<< target_size.y  << "      "<< ttk::pose2str3D(target_pose) << "      "<< target_pose.header.frame_id);
  return s * 0.80;
}

};
