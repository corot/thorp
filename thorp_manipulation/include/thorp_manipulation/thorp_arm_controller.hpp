/*
 * Author: Jorge Santos
 */

#pragma once

#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Thorp messages
#include <thorp_msgs/ThorpError.h>


namespace thorp_manipulation
{

/**
 * Base class for all controllers intended to operate Thorp's arm.
 * Note that we use the Construct On First Use idiom to allow using static attributes for move groups while
 * avoiding the static initialization disaster (these attributes cannot be initialized until ROS is running).
 */
class ThorpArmController
{
public:
  ThorpArmController()
  {
    ros::NodeHandle nh, pnh("~");

    // Read arm control parameters
    pnh.param("arm_ctrl_ref_frame", arm_ref_frame, std::string("arm_base_link"));
    pnh.param("vertical_backlash_scale", vertical_backlash_scale, 0.0);
    pnh.param("vertical_backlash_delta", vertical_backlash_delta, 0.0);
    pnh.param("fall_short_distance_delta", fall_short_distance_delta, 0.0);
    pnh.param("gripper_asymmetry_yaw_delta", gripper_asymmetry_yaw_delta, 0.0);

    nh.param("gripper_controller/max_opening", gripper_open, 0.0454321);

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
  moveit::planning_interface::MoveGroupInterface& arm()
  {
    static moveit::planning_interface::MoveGroupInterface arm("arm");
    return arm;
  }

  moveit::planning_interface::MoveGroupInterface& gripper()
  {
    static moveit::planning_interface::MoveGroupInterface gripper("gripper");
    return gripper;
  }

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world   TODO move to toolkit to make common with other servers!
  moveit::planning_interface::PlanningSceneInterface& planning_scene_interface()
  {
    static moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    return planning_scene_interface;
  }

  static std::string attached_object;

  // Pick and place parameters
  std::string arm_ref_frame;
  double      gripper_open;
  double      vertical_backlash_scale;
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
   * @param compensate_vertical_backlash Increment z to cope with backlash and low pitch poses
   * @param compensate_gripper_asymmetry Increment the yaw to compensate that only the right
   *        finger opens, and so there's more grasping room on the right
   * @param compensate_distance_fall_short Increment distance... no justification, really; it
   *        just put the target in the center of the gripper!
   * @param attempt The actual attempts number
   * @return True of success, false otherwise
   */
  bool validateTargetPose(geometry_msgs::PoseStamped& target, bool compensate_vertical_backlash,
                          bool compensate_gripper_asymmetry, bool compensate_distance_fall_short,
                          int attempt = 0);

  /**
   * Set gripper opening.
   * @param opening Physical opening of the gripper, in meters
   * @param wait_for_complete Wait or not for the execution of the trajectory to complete
   * @return True of success, false otherwise
   */
  bool setGripper(float opening, bool wait_for_complete = true);

  /**
   * Provide a meaningful text for each MoveIt error code.
   * @param mec MoveIt error code
   * @return meaningful text
   */
  inline const char* mec2str(const moveit::planning_interface::MoveItErrorCode& mec)
  {
    return mec2str(mec.val);
  }

  /**
   * Provide a meaningful text for each Thorp error code.
   * @param error Thorp error code
   * @return meaningful text
   */
  inline const char* mec2str(const thorp_msgs::ThorpError& error)
  {
    return mec2str(error.code);
  }

  /**
   * Provide a meaningful text for MoveIt, pick, place, move arm, etc. error codes.
   * @param error_code MoveIt, pick, place, etc. error code
   * @return meaningful text
   */
  inline const char* mec2str(int32_t error_code)
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
      case thorp_msgs::ThorpError::OBJECT_NOT_ATTACHED:
        return "requested object is not attached";
      case thorp_msgs::ThorpError::OBJECT_ATTACHED:
        return "already have an object attached";

      default:
        return "unrecognized error code";
    }
  }

  inline float fRand(float min, float max)
  {
    return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
  }
};

};
