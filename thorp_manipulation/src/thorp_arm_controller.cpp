/*
 * Author: Jorge Santos
 */

// auxiliary libraries
#include <thorp_toolkit/tf2.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;

#include "thorp_manipulation/thorp_arm_controller.hpp"

namespace thorp_manipulation
{

std::string ThorpArmController::attached_object;


bool ThorpArmController::setGripper(float opening, bool wait_for_complete)
{
  ROS_DEBUG("[arm controller] Set gripper opening to %f", opening);
  if (!gripper().setJointValueTarget("gripper_joint", opening))
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

bool ThorpArmController::validateTargetPose(geometry_msgs::PoseStamped& target, bool compensate_vertical_backlash,
                                            bool compensate_gripper_asymmetry, bool compensate_distance_fall_short,
                                            int attempt)
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
    if (!ttk::TF2::transformPose(target.header.frame_id, arm_ref_frame, target, target))
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
  //                in RPY (degree) [0.000, 0.000, 0.000]  TODO use tf!  ttk::transformPose(arm_ref_frame, "arm_shoulder_lift_servo_link");
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
  double ry = ttk::heading(target.pose);
  double rr = 0.0;
  target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rr, rp, ry);

  // Rather ad hoc compensations for our arm deficiencies:
  if (compensate_vertical_backlash)
  {
    // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner and
    // a bit extra to compensate the effect of the arm's backlash in the height of the gripper over the table
    double z_delta1 = vertical_backlash_scale * (1.0 - std::abs(std::sin(rp)))/(M_PI*M_PI);
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

  ROS_DEBUG("[arm controller] Target pose [%s] [d: %.2f]", ttk::pose2cstr3D(target.pose), d);
  target_pose_pub.publish(target);

  return true;
}

};
