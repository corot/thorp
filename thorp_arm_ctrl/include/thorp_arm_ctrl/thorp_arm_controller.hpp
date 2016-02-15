/*
 * Copyright (c) 2016, Jorge Santos
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

#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// auxiliary libraries
#include <yocs_math_toolkit/common.hpp>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


namespace thorp_arm_ctrl
{

/**
 * Base class for all controllers intended to operate Thorp's arm.
 * Note that we use the Construct On First Use idiom to allow using static attributes for move groups and
 * planning scene interface while avoiding the static initialization disaster (the three attributes cannot
 * be initialized until ROS is up and running).
 */
class ThorpArmController
{
public:
  ThorpArmController()
  {
    ros::NodeHandle nh("~");

    // Read arm control parameters
    nh.param("arm_ctrl_ref_frame", arm_ref_frame, std::string("arm_base_link"));
    nh.param("grasp_attach_time", attach_time, 0.8);
    nh.param("grasp_detach_time", detach_time, 0.6);
    nh.param("vertical_backlash", z_backlash, 0.01);
    nh.param("/gripper_controller/max_opening", gripper_open, 0.045);

    // Default target poses reference frame: we normally work relative to
    // the arm base, so our calculated roll/pitch/yaw angles make sense
    arm().setPoseReferenceFrame(arm_ref_frame);
  }

  ~ThorpArmController()
  {
  }

protected:
    // Move groups to control arm and gripper with MoveIt!
    moveit::planning_interface::MoveGroup& arm()
    {
      static moveit::planning_interface::MoveGroup arm("arm");
      return arm;
    }

    moveit::planning_interface::MoveGroup& gripper()
    {
      static moveit::planning_interface::MoveGroup gripper("gripper");
      return gripper;
    }

    // We use the planning scene to gather information of tabletop/attached objects
    moveit::planning_interface::PlanningSceneInterface& planningScene()
    {
      static moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      return planning_scene_interface;
    }

    // Pick and place parameters
    std::string arm_ref_frame;
    double      gripper_open;
    double      attach_time;
    double      detach_time;
    double      z_backlash;


  /**
   * Convert a simple 3D point into a valid pick/place pose. The orientation Euler angles
   * are calculated as a function of the x and y coordinates, plus some random variations
   * increasing with the number of attempts to improve our chances of successful planning.
   * @param target Pose target to validate
   * @param compensate_backlash Increment z to cope with backlash and low pitch poses
   * @param attempt The actual attempts number
   * @return True of success, false otherwise
   */
  bool validateTargetPose(geometry_msgs::PoseStamped& target, bool compensate_backlash, int attempt = 0)
  {
    static ros::Publisher target_pose_pub;
    if (target_pose_pub.getTopic().empty())
    {
      // We publish the pick and place poses for debugging purposes; advertise topic just once!
      ros::NodeHandle nh;
      target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1, true);
    }

    // We always work relative to the arm base, so roll/pitch/yaw angles calculation make sense
    if (target.header.frame_id != arm_ref_frame)
    {
      transformPose(target.header.frame_id, arm_ref_frame, target, target);
    }

    double x = target.pose.position.x;
    double y = target.pose.position.y;
    double z = target.pose.position.z;
    double d = sqrt(x*x + y*y);
    if (d > 0.3)
    {
      // Maximum reachable distance by the turtlebot arm is 30 cm, but above twenty something the arm makes
      // strange and ugly contortions, and overcomes the reduced elbow lower limit we have to operate always
      // with the same gripper orientation
      // XXX solved constraining also both shoulder limits (180 deg. operation); we get back the 30 cm limit
      ROS_ERROR("[pick and place] Target pose out of reach [%f > %f]", d, 0.3);
      return false;
    }

    // Pitch is 90 (vertical) at 10 cm from the arm base; the farther the target is, the closer to horizontal
    // we point the gripper (0.22 = arm's max reach - vertical pitch distance + ε). We also try some random
    // variations to increase the chances of successful planning. Yaw is the direction to the target, and so
    // must be fixed. Roll is plainly ignored, as our arm lacks that dof.
    double rp = M_PI_2 - std::asin((d - 0.1)/0.22) + ((attempt%2)*2 - 1)*(std::ceil(attempt/2.0)*0.05);
    double ry = std::atan2(y, x);
    double rr = 0.0;
    target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rr, rp, ry);

    if (compensate_backlash)
    {
      // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner and
      // a bit extra to compensate the effect of the arm's backlash in the height of the gripper over the table
      double z_delta1 = std::abs(std::cos(rp))/50.0;
      double z_delta2 = z_backlash;
      ROS_DEBUG("[pick and place] Z increase:  %f  +  %f  +  %f", target.pose.position.z, z_delta1, z_delta2);
      target.pose.position.z += z_delta1;
      target.pose.position.z += z_delta2;
    }

    ROS_DEBUG("[pick and place] Target pose [%s] [d: %.2f]", mtk::pose2str3D(target.pose).c_str(), d);
    target_pose_pub.publish(target);

    return true;
  }

  /**
   * Provide a meaningful text for each MoveIt error code.
   * @param mec MoveIt error code
   * @return meaningful text
   */
  const char* mec2str(const moveit::planning_interface::MoveItErrorCode& mec)
  {
    switch (mec.val)
    {
      case moveit::planning_interface::MoveItErrorCode::SUCCESS:
        return "success";
      case moveit::planning_interface::MoveItErrorCode::FAILURE:
        return "failure";
      case moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED:
        return "planning failed";
      case moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN:
        return "invalid motion plan";
      case moveit::planning_interface::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return "motion plan invalidated by environment change";
      case moveit::planning_interface::MoveItErrorCode::CONTROL_FAILED:
        return "control failed";
      case moveit::planning_interface::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
        return "unable to acquire sensor data";
      case moveit::planning_interface::MoveItErrorCode::TIMED_OUT:
        return "timed out";
      case moveit::planning_interface::MoveItErrorCode::PREEMPTED:
        return "preempted";
      case moveit::planning_interface::MoveItErrorCode::START_STATE_IN_COLLISION:
        return "start state in collision";
      case moveit::planning_interface::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "start state violates path constraints";
      case moveit::planning_interface::MoveItErrorCode::GOAL_IN_COLLISION:
        return "goal in collision";
      case moveit::planning_interface::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "goal violates path constraints";
      case moveit::planning_interface::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED:
        return "goal constraints violated";
      case moveit::planning_interface::MoveItErrorCode::INVALID_GROUP_NAME:
        return "invalid group name";
      case moveit::planning_interface::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
        return "invalid goal constraints";
      case moveit::planning_interface::MoveItErrorCode::INVALID_ROBOT_STATE:
        return "invalid robot state";
      case moveit::planning_interface::MoveItErrorCode::INVALID_LINK_NAME:
        return "invalid link name";
      case moveit::planning_interface::MoveItErrorCode::INVALID_OBJECT_NAME:
        return "invalid object name";
      case moveit::planning_interface::MoveItErrorCode::FRAME_TRANSFORM_FAILURE:
        return "frame transform failure";
      case moveit::planning_interface::MoveItErrorCode::COLLISION_CHECKING_UNAVAILABLE:
        return "collision checking unavailable";
      case moveit::planning_interface::MoveItErrorCode::ROBOT_STATE_STALE:
        return "robot state stale";
      case moveit::planning_interface::MoveItErrorCode::SENSOR_INFO_STALE:
        return "sensor info stale";
      case moveit::planning_interface::MoveItErrorCode::NO_IK_SOLUTION:
        return "no ik solution";
      default:
        return "unrecognized error code";
    }
  }


  bool transformPose(const std::string& in_frame, const std::string& out_frame,
                     const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose)
  {
    try
    {
      static tf::TransformListener tf_listener_;
      tf_listener_.waitForTransform(in_frame, out_frame, ros::Time(0.0), ros::Duration(1.0));
      tf_listener_.transformPose(out_frame, in_pose, out_pose);

      return true;
    }
    catch (tf::InvalidArgument& e)
    {
      ROS_ERROR("[pick and place] Transformed pose has invalid orientation: %s", e.what());
      return false;
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR("[pick and place] Could not get sensor to arm transform: %s", e.what());
      return false;
    }
  }

  /**
   * Set gripper opening.
   * @param opening Physical opening of the gripper, in meters
   * @param wait_for_complete Wait or not for the execution of the trajectory to complete
   * @return True of success, false otherwise
   */
  bool setGripper(float opening, bool wait_for_complete = true)
  {
    ROS_DEBUG("[move to target] Set gripper opening to %f", opening);
    if (gripper().setJointValueTarget("gripper_joint", opening) == false)
    {
      ROS_ERROR("[move to target] Set gripper opening to %f failed", opening);
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result =
        wait_for_complete ? gripper().move() : gripper().asyncMove();
    if (result == true)
    {
      return true;
    }
    else
    {
      ROS_ERROR("[move to target] Set gripper opening failed (error %d)", result.val);
      return false;
    }
  }

  float fRand(float min, float max)
  {
    return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
  }
};

};
