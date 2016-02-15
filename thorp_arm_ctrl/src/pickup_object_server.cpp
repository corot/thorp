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

#include <ros/ros.h>
#include <tf/tf.h>

// auxiliary libraries
#include <yocs_math_toolkit/common.hpp>
#include <geometric_shapes/shape_operations.h>

// MoveIt!
#include <moveit_msgs/Grasp.h>


#include "thorp_arm_ctrl/pickup_object_server.hpp"


namespace thorp_arm_ctrl
{

PickupObjectServer::PickupObjectServer(const std::string name) :
  as_(name, false), action_name_(name)
{
//  ros::NodeHandle nh("~");
//
//  // Read specific pick and place parameters
//  nh.param("grasp_attach_time", attach_time, 0.8);
//  nh.param("grasp_detach_time", detach_time, 0.6);
//  nh.param("vertical_backlash", z_backlash, 0.01);
//  nh.param("/gripper_controller/max_opening", gripper_open, 0.045);

  // Register the goal and feedback callbacks
  as_.registerGoalCallback(boost::bind(&PickupObjectServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PickupObjectServer::preemptCB, this));
  as_.start();
}

PickupObjectServer::~PickupObjectServer()
{
  as_.shutdown();
}

void PickupObjectServer::goalCB()
{
  ROS_INFO("[pickup object] Received goal!");

  goal_ = as_.acceptNewGoal();

  arm().setSupportSurfaceName(goal_->support_surf);

  // Allow some leeway in position (meters) and orientation (radians)
  arm().setGoalPositionTolerance(0.001);
  arm().setGoalOrientationTolerance(0.02);

  // Allow replanning to increase the odds of a solution
  arm().allowReplanning(true);

  if (pickup(goal_->object_name, goal_->support_surf))
  {
    as_.setSucceeded(result_);
  }
  else
  {
    as_.setAborted(result_);
  }
}

void PickupObjectServer::preemptCB()
{
  ROS_WARN("[pickup object] %s: Preempted", action_name_.c_str());
  gripper().stop();
  arm().stop();

  // set the action state to preempted
  as_.setPreempted();
}

bool PickupObjectServer::pickup(const std::string& obj_name, const std::string& surface)
{
  // Look for obj_name in the list of available objects
  std::map<std::string, moveit_msgs::CollisionObject> objects =
      planningScene().getObjects(std::vector<std::string>(1, obj_name));
  if (objects.size() == 0)
  {
    // Maybe the object's interactive marker name is wrong?
    ROS_ERROR("[pickup object] Tabletop collision object '%s' not found", obj_name.c_str());
    return false;
  }

  if (objects.size() > 1)
  {
    // This should not happen, as object detection tries to provide unique names to all objects...
    ROS_WARN("[pickup object] More than one (%d) tabletop collision objects with name '%s' found!",
             objects.size(), obj_name.c_str());
  }

  // We need object's pose and size for picking
  Eigen::Vector3d tco_size;
  geometry_msgs::PoseStamped tco_pose;
  const moveit_msgs::CollisionObject& tco = objects[obj_name];

  tco_pose.header = tco.header;

  // We get object's pose from the mesh/primitive poses; try first with the meshes
  if (tco.mesh_poses.size() > 0)
  {
    tco_pose.pose = tco.mesh_poses[0];
    if (tco.meshes.size() > 0)
    {
      tco_size = shapes::computeShapeExtents(tco.meshes[0]);

      // We assume meshes laying in the floor, so we bump its pose by half z-dimension to
      // grasp the object at mid-height. TODO: we could try something more sophisticated...
      tco_pose.pose.position.z += tco_size[2]/2.0;
    }
    else
    {
      ROS_ERROR("[pickup object] Tabletop collision object '%s' has no meshes", obj_name.c_str());
      return false;
    }
  }
  else if (tco.primitive_poses.size() > 0)
  {
    tco_pose.pose = tco.primitive_poses[0];
    if (tco.primitives.size() > 0)
    {
      tco_size = shapes::computeShapeExtents(tco.primitives[0]);
    }
    else
    {
      ROS_ERROR("[pickup object] Tabletop collision object '%s' has no primitives", obj_name.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("[pickup object] Tabletop collision object '%s' has no mesh/primitive poses", obj_name.c_str());
    return false;
  }

  ROS_INFO("[pickup object] Picking object '%s' with size [%.3f, %.3f, %.3f] at location [%s]...",
           obj_name.c_str(), tco_size[0], tco_size[1], tco_size[2], mtk::point2str2D(tco_pose.pose.position).c_str());

  // Try up to PICK_ATTEMPTS grasps with slightly different poses
  for (int attempt = 0; attempt < PICK_ATTEMPTS; ++attempt)
  {
    geometry_msgs::PoseStamped p = tco_pose;
    if (!validateTargetPose(p, true, attempt))
    {
      return false;
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
    g.grasp_posture.points[0].positions.push_back(tco_size.minCoeff() - 0.002);

    g.allowed_touch_objects.push_back(obj_name);
    g.allowed_touch_objects.push_back(surface);

    g.id = attempt;

    std::vector<moveit_msgs::Grasp> grasps(1, g);

    moveit::planning_interface::MoveItErrorCode result = arm().pick(obj_name, grasps);
    if (result)
    {
      ROS_INFO("[pickup object] Pick successfully completed");
      return true;
    }

    ROS_DEBUG("[pickup object] Pick attempt %d failed: %s", attempt, mec2str(result));
  }

  ROS_ERROR("[pickup object] Pick failed after %d attempts", PICK_ATTEMPTS);
  return false;
}

};
