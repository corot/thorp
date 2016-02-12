/*
 * Copyright (c) 2015, Jorge Santos
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Vanadium Labs LLC nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Jorge Santos
 */

#include <tf/tf.h>
#include <ros/ros.h>

// auxiliary libraries
#include <yocs_math_toolkit/common.hpp>
#include <geometric_shapes/shape_operations.h>

// MoveIt!
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>

#include "thorp_arm_ctrl/place_object_server.hpp"


namespace thorp_arm_ctrl
{

PlaceObjectServer::PlaceObjectServer(const std::string name) :
  as_(name, false), action_name_(name)
{
  // Register the goal and feedback callbacks
  as_.registerGoalCallback(boost::bind(&PlaceObjectServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlaceObjectServer::preemptCB, this));
  as_.start();
}

PlaceObjectServer::~PlaceObjectServer()
{
  as_.shutdown();
}

void PlaceObjectServer::goalCB()
{
  ROS_INFO("[place object] Received goal!");

  goal_ = as_.acceptNewGoal();

  arm_.setSupportSurfaceName(goal_->support_surf);

  // Allow some leeway in position (meters) and orientation (radians)
  arm_.setGoalPositionTolerance(0.001);
  arm_.setGoalOrientationTolerance(0.02);

  // Allow replanning to increase the odds of a solution
  arm_.allowReplanning(true);

  if (place(goal_->object_name, goal_->support_surf, goal_->place_pose))
  {
    as_.setSucceeded(result_);
  }
  else
  {
    // Ensure we don't retain any object attached to the gripper
    arm_.detachObject(goal_->object_name);
    setGripper(gripper_open, false);

    as_.setAborted(result_);
  }
}

void PlaceObjectServer::preemptCB()
{
  ROS_WARN("[place object] %s: Preempted", action_name_.c_str());
  gripper_.stop();
  arm_.stop();

  // set the action state to preempted
  as_.setPreempted();
}

bool PlaceObjectServer::place(const std::string& obj_name, const std::string& surface,
                              const geometry_msgs::PoseStamped& pose)
{
  // Look for obj_name in the list of attached objects
  std::map<std::string, moveit_msgs::AttachedCollisionObject> objects =
      planning_scene_interface_.getAttachedObjects(std::vector<std::string>(1, obj_name));

  if (objects.size() == 0)
  {
    // Maybe pick failed; we will not continue because place will surely fail without knowing the attaching pose
    ROS_ERROR("[place object] Attached collision object '%s' not found", obj_name.c_str());
    return false;
  }

  if (objects.size() > 1)
  {
    // This should not happen... we grasped two objects with the same name???
    ROS_WARN("[place object] More than one (%d) attached collision objects with name '%s' found!",
             objects.size(), obj_name.c_str());
  }

  // We just need object's pose so we can subtract its pose from the place location poses
  geometry_msgs::Pose aco_pose;  // No stamped; it's relative to attaching frame (gripper_link)
  const moveit_msgs::AttachedCollisionObject& aco = objects[obj_name];

  if (aco.object.primitive_poses.size() > 0)
  {
    aco_pose = aco.object.primitive_poses[0];
  }
  else if (aco.object.mesh_poses.size() > 0)
  {
    aco_pose = aco.object.mesh_poses[0];
  }
  else
  {
    ROS_ERROR("[place object] Attached collision object '%s' has no pose!", obj_name.c_str());
    return false;
  }

  ROS_INFO("[place object] Placing object '%s' at pose [%s]...", obj_name.c_str(), mtk::pose2str3D(pose).c_str());

  // Try up to PLACE_ATTEMPTS place locations with slightly different poses
  for (int attempt = 0; attempt < PLACE_ATTEMPTS; ++attempt)
  {
    geometry_msgs::PoseStamped p = pose;
    if (!validateTargetPose(p, true, attempt))
    {
      return false;
    }

    // MoveGroup::place will transform the provided place pose with the attached body pose, so the object retains
    // the orientation it had when picked. However, with our 4-dofs arm this is infeasible (nor we care about the
    // objects orientation!), so we cancel this transformation. It is applied here:
    // https://github.com/ros-planning/moveit_ros/blob/jade-devel/manipulation/pick_place/src/place.cpp#L64
    // More details on this issue: https://github.com/ros-planning/moveit_ros/issues/577
    tf::Transform place_tf, aco_tf;
    tf::poseMsgToTF(p.pose, place_tf);
    tf::poseMsgToTF(aco_pose, aco_tf);
    tf::poseTFToMsg(place_tf * aco_tf, p.pose);

    ROS_DEBUG("Compensate place pose with the attached object pose [%s]. Results: [%s]",
              mtk::pose2str3D(aco_pose).c_str(), mtk::pose2str3D(p.pose).c_str());

    ROS_DEBUG("[place object] Place attempt %d at pose [%s]...", attempt, mtk::pose2str3D(p).c_str());

    moveit_msgs::PlaceLocation l;
    l.place_pose = p;

    l.pre_place_approach.direction.vector.x = 0.5;
    l.pre_place_approach.direction.header.frame_id = arm_.getEndEffectorLink();
    l.pre_place_approach.min_distance = 0.005;
    l.pre_place_approach.desired_distance = 0.1;

    l.post_place_retreat.direction.vector.x = -0.5;
    l.post_place_retreat.direction.header.frame_id = arm_.getEndEffectorLink();
    l.post_place_retreat.min_distance = 0.005;
    l.post_place_retreat.desired_distance = 0.1;

    l.post_place_posture.joint_names.push_back("gripper_joint");
    l.post_place_posture.points.resize(1);
    l.post_place_posture.points[0].positions.push_back(gripper_open);

    l.allowed_touch_objects.push_back(obj_name);
    l.allowed_touch_objects.push_back(surface);

    l.id = attempt;

    std::vector<moveit_msgs::PlaceLocation> locs(1, l);

    moveit::planning_interface::MoveItErrorCode result = arm_.place(obj_name, locs);
    if (result)
    {
      ROS_INFO("[place object] Place successfully completed");
      return true;
    }

    ROS_DEBUG("[place object] Place attempt %d failed: %s", attempt, mec2str(result));
  }

  ROS_ERROR("[place object] Place failed after %d attempts", PLACE_ATTEMPTS);
  return false;
}

};
