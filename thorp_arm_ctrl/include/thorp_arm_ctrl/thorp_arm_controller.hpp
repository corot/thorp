/*
 * Author: Jorge Santos
 */

#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// auxiliary libraries
#include <thorp_toolkit/common.hpp>
#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Thorp messages
#include <thorp_msgs/ThorpError.h>


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
    nh.param("vertical_backlash_delta", vertical_backlash_delta, 0.01);
    nh.param("fall_short_distance_delta", fall_short_distance_delta, 0.008);
    nh.param("gripper_asymmetry_yaw_delta", gripper_asymmetry_yaw_delta, 0.05);

    nh.param("/gripper_controller/max_opening", gripper_open, 0.045);

    // Default target poses reference frame: we normally work relative to
    // the arm base, so our calculated roll/pitch/yaw angles make sense
    arm().setPoseReferenceFrame(arm_ref_frame);

    // Allow some leeway in position (meters) and orientation (radians)
    arm().setGoalPositionTolerance(0.001);
    arm().setGoalOrientationTolerance(0.02);

    // Allow replanning to increase the odds of a solution
    arm().allowReplanning(true);
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
    double      vertical_backlash_delta;
    double      gripper_asymmetry_yaw_delta;
    double      fall_short_distance_delta;

    // Arm's physical reach limitations
    double const MAX_DISTANCE = 0.30;
    double const MAX_HEIGHT   = 0.20;


  /**
   * Convert a simple 3D point into a valid pick/place pose. The orientation Euler angles
   * are calculated as a function of the x and y coordinates, plus some random variations
   * increasing with the number of attempts to improve our chances of successful planning.
   * @param target Pose target to validate
   * @param compensate_backlash Increment z to cope with backlash and low pitch poses
   * @param attempt The actual attempts number
   * @return True of success, false otherwise
   */
  bool validateTargetPose(geometry_msgs::PoseStamped& target, bool compensate_vertical_backlash,
                          bool compensate_gripper_asymmetry, bool compensate_distance_fall_short,
                          int attempt = 0)
  {
    static ros::Publisher target_pose_pub;
    if (target_pose_pub.getTopic().empty())
    {
      // We publish the pick and place poses for debugging purposes; advertise topic just once!
      ros::NodeHandle nh;
      target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1, true);
    }

    // We always work relative to the arm base, so we can calculate meaningful roll/pitch/yaw angles; as
    // given values are ignored, replace them with an identity quaternion to allow position-only targets
    if (target.header.frame_id != arm_ref_frame)
    {
      // Target's timestamp is irrelevant, and can trigger a TransformException if very recent; zero it!
      target.header.stamp = ros::Time(0.0);
      tf::quaternionTFToMsg(tf::createIdentityQuaternion(), target.pose.orientation);
      if (!thorp_toolkit::transformPose(target.header.frame_id, arm_ref_frame, target, target))
        return false;
    }

    // We work on arm_ref_frame, but it's useful to set z relative to arm_shoulder_lift_servo_link, to
    // calculate the target 3D distance and the high target correction, as that's the arm's operation
    // plane (the height at which it can reach further), 6cm above arm_base_link. I got it with:
    //    rosrun  tf tf_echo /arm_base_link /arm_shoulder_lift_servo_link
    //    At time 1456156899.841
    //    - Translation: [0.000, -0.000, 0.060]
    //    - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
    //                in RPY (radian) [0.000, 0.000, 0.000]
    //                in RPY (degree) [0.000, 0.000, 0.000]
    double x = target.pose.position.x;
    double y = target.pose.position.y;
    double z = target.pose.position.z - 0.06;
    double d = sqrt(x*x + y*y + z*z);
    if (d > MAX_DISTANCE)
    {
      // Maximum reachable distance by the turtlebot arm is 30 cm, but above twenty something the arm makes
      // strange and ugly contortions, and overcomes the reduced elbow lower limit we have to operate always
      // with the same gripper orientation
      // XXX solved constraining also both shoulder limits (180 deg. operation); we get back the 30 cm limit
      ROS_ERROR("[arm controller] Target pose out of reach [%f > %f]", d, MAX_DISTANCE);
      return false;
    }
    if (std::abs(z) > MAX_HEIGHT)
    {
      // Maximum reachable height by the turtlebot arm is around +/-20 cm from arm_shoulder_lift_servo_link
      ROS_ERROR("[arm controller] Target pose out of reach [%f > %f]", std::abs(z), MAX_HEIGHT);
      return false;
    }

    // Pitch is 90 (vertical) at 10 cm from the arm base; the farther the target is, the closer to horizontal
    // we point the gripper (0.22 = arm's max reach - vertical pitch distance + ε). We also add a correction
    // when trying to reach targets above the arm operation plane, ranging from zero correction to set the
    // gripper horizontal when reaching a target at MAX_HEIGHT (I never tried it!). Finally, try some random
    // variations to increase the chances of successful planning. Yaw is the direction to the target, and so
    // must be fixed. Roll is plainly ignored, as our arm lacks that dof.
    double pitch_delta1 = (z > 0.0 ? -M_PI_2*(z/MAX_HEIGHT) : 0.0);
    double pitch_delta2 = ((attempt%2)*2 - 1)*(std::ceil(attempt/2.0)*0.05);
    ROS_DEBUG("[arm controller] Pitch high target correction: %f;  random variation: %f", pitch_delta1, pitch_delta2);

    double rp = (M_PI_2 - std::asin((d - 0.1)/0.22)) + pitch_delta1 + pitch_delta2;
    double ry = mtk::heading(target.pose);
    double rr = 0.0;
    target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rr, rp, ry);

    // Rather ad hoc compensations for our arm deficiencies:
    if (compensate_vertical_backlash)
    {
      // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner and
      // a bit extra to compensate the effect of the arm's backlash in the height of the gripper over the table
      double z_delta1 = (1.0 - std::abs(std::sin(rp)))/(M_PI*M_PI);
      double z_delta2 = vertical_backlash_delta;
      ROS_DEBUG("[arm controller] Z increases:  %f  +  %f  +  %f", target.pose.position.z, z_delta1, z_delta2);
      target.pose.position.z += z_delta1;
      target.pose.position.z += z_delta2;
    }

    if (compensate_gripper_asymmetry)
    {
      // Slightly increase the yaw because only the right finger opens, and so there's more grasping room on the right
      target.pose.position.x = d*std::cos(ry + gripper_asymmetry_yaw_delta);
      target.pose.position.y = d*std::sin(ry + gripper_asymmetry_yaw_delta);
      target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rr, rp, ry + gripper_asymmetry_yaw_delta);
      ROS_DEBUG("[arm controller] Compensate gripper asymmetry increasing yaw by %frad", gripper_asymmetry_yaw_delta);
    }

    if (compensate_distance_fall_short)
    {
      // Slightly increase distance... no justification, really. It just put the target in the center of the gripper!
      target.pose.position.x += fall_short_distance_delta*std::cos(ry);
      target.pose.position.y += fall_short_distance_delta*std::sin(ry);
      ROS_DEBUG("[arm controller] Compensate distance fall short increasing distance by %fm", fall_short_distance_delta);
    }

    ROS_DEBUG("[arm controller] Target pose [%s] [d: %.2f]", mtk::pose2str3D(target.pose).c_str(), d);
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
    return mec2str(mec.val);
  }

  /**
   * Provide a meaningful text for each Thorp error code.
   * @param error Thorp error code
   * @return meaningful text
   */
  const char* mec2str(const thorp_msgs::ThorpError& error)
  {
    return mec2str(error.code);
  }

  /**
   * Provide a meaningful text for MoveIt, pick, place, move arm, etc. error codes.
   * @param error_code MoveIt, pick, place, etc. error code
   * @return meaningful text
   */
  const char* mec2str(int32_t error_code)
  {
    switch (error_code)
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

      case thorp_msgs::ThorpError::OBJECT_NOT_FOUND:
        return "object not found";
      case thorp_msgs::ThorpError::OBJECT_POSE_NOT_FOUND:
        return "object pose not found";
      case thorp_msgs::ThorpError::OBJECT_SIZE_NOT_FOUND:
        return "object meshes or primitives not found";
      case thorp_msgs::ThorpError::INVALID_TARGET_POSE:
        return "invalid target pose";
      case thorp_msgs::ThorpError::INVALID_TARGET_TYPE:
        return "invalid target type";
      case thorp_msgs::ThorpError::INVALID_NAMED_TARGET:
        return "invalid named target";
      case thorp_msgs::ThorpError::INVALID_JOINT_STATE:
        return "invalid joint state";
      case thorp_msgs::ThorpError::SERVER_NOT_AVAILABLE:
        return "server not available";

      default:
        return "unrecognized error code";
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
    ROS_DEBUG("[arm controller] Set gripper opening to %f", opening);
    if (gripper().setJointValueTarget("gripper_joint", opening) == false)
    {
      ROS_ERROR("[arm controller] Set gripper opening to %f failed", opening);
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
      ROS_ERROR("[arm controller] Set gripper opening failed (error %d)", result.val);
      return false;
    }
  }

  float fRand(float min, float max)
  {
    return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
  }
};

};
