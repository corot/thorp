/*
 * Author: Jorge Santos
 */

#pragma once

#include <optional>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace thorp::manipulation
{
/**
 * Ad hoc corrections for the real arm's deficiencies.
 */
struct ArmCompensations
{
  double vertical_backlash_scale = 0.0;      // raise low pitch targets, to not hit the table with the gripper corner
  double vertical_backlash_delta = 0.0;      // raise all targets, as backlash makes the gripper fall short in height
  double fall_short_distance_delta = 0.0;    // extend targets distance, as that centers them on the gripper
  double gripper_asymmetry_yaw_delta = 0.0;  // turn targets to the right, as only the right finger opens
};

/**
 * One-sided gripper model: converts openings between the fingers pads into gripper joint angles.
 */
struct GripperModel
{
  double center = 0.07;
  double pad_width = 0.025;
  double finger_length = 0.03;

  double angle(double opening) const;
};

/**
 * Turn a target position into a pose the arm can reach with the gripper. Yaw points to the target, as the arm
 * lacks a roll joint, and pitch goes from vertical at 10 cm from the arm base to horizontal at full reach, with a
 * correction for targets above the arm operation plane and small variations increasing with the attempt number.
 * @param target Target position, relative to the arm base
 * @param operation_height Height of the arm operation plane (the shoulder lift servo) over the arm base
 * @param compensations Corrections for the arm deficiencies
 * @param attempt Attempt number; each one gives a slightly different pitch
 * @param error Reason for rejecting the target
 * @return The target pose, or nothing if the target is out of reach
 */
std::optional<geometry_msgs::msg::Pose> makeTargetPose(const geometry_msgs::msg::Point& target,
                                                       double operation_height,
                                                       const ArmCompensations& compensations, int attempt,
                                                       std::string& error);

/**
 * Gripper opening to grasp an object blindly on its center: the object side more across the gripper, minus some
 * tightening. All yaws must be relative to the same frame.
 * @param grasp_yaw Yaw of the grasp pose
 * @param object_yaw Yaw of the object
 * @param object_size Object bounding box size
 * @param tightening Opening reduction to tighten the grasp
 */
double graspOpening(double grasp_yaw, double object_yaw, const geometry_msgs::msg::Vector3& object_size,
                    double tightening);

}  // namespace thorp::manipulation
