/*
 * Author: Jorge Santos
 */

#include "thorp_manipulation/arm_targets.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace thorp::manipulation
{
// Arm's physical reach: distance from the shoulder lift servo, and height over or under it
constexpr double MAX_DISTANCE = 0.30;
constexpr double MAX_HEIGHT = 0.20;

double GripperModel::angle(double opening) const
{
  // The active finger closes with positive angles
  return center - std::asin((opening - pad_width) / (2.0 * finger_length));
}

double GripperModel::opening(double angle) const
{
  return pad_width + 2.0 * finger_length * std::sin(center - angle);
}

std::optional<geometry_msgs::msg::Pose> makeTargetPose(const geometry_msgs::msg::Point& target,
                                                       double operation_height,
                                                       const ArmCompensations& compensations, int attempt,
                                                       std::string& error)
{
  double x = target.x;
  double y = target.y;
  double z = target.z - operation_height;
  double d = std::sqrt(x * x + y * y + z * z);
  if (d > MAX_DISTANCE)
  {
    error = "target out of reach: distance " + std::to_string(d) + " > " + std::to_string(MAX_DISTANCE);
    return std::nullopt;
  }
  if (std::abs(z) > MAX_HEIGHT)
  {
    error = "target out of reach: height " + std::to_string(z) + " beyond +/-" + std::to_string(MAX_HEIGHT);
    return std::nullopt;
  }

  double high_target_correction = z > 0.0 ? -M_PI_2 * (z / MAX_HEIGHT) : 0.0;
  double attempt_variation = ((attempt % 2) * 2 - 1) * (std::ceil(attempt / 2.0) * 0.05);  // 0, +/- increasing
  double d2d = std::sqrt(x * x + y * y);
  // 0.22 is the arm's max reach minus the vertical pitch distance, plus a margin
  double pitch = (M_PI_2 - std::asin((d2d - 0.1) / 0.22)) + high_target_correction + attempt_variation;
  double yaw = std::atan2(y, x);

  geometry_msgs::msg::Pose pose;
  pose.position = target;
  pose.position.z += compensations.vertical_backlash_scale * (1.0 - std::abs(std::sin(pitch))) / (M_PI * M_PI);
  pose.position.z += compensations.vertical_backlash_delta;
  double turned_yaw = yaw + compensations.gripper_asymmetry_yaw_delta;
  pose.position.x = d2d * std::cos(turned_yaw) + compensations.fall_short_distance_delta * std::cos(yaw);
  pose.position.y = d2d * std::sin(turned_yaw) + compensations.fall_short_distance_delta * std::sin(yaw);
  tf2::Quaternion q;
  q.setRPY(0.0, pitch, turned_yaw);
  pose.orientation = tf2::toMsg(q);
  return pose;
}

double graspOpening(double grasp_yaw, double object_yaw, const geometry_msgs::msg::Vector3& object_size,
                    double tightening)
{
  // Angle between the gripper and the object x axis, normalized to [0, pi/2]; the fingers close across the object
  // y side if the gripper is more aligned with its x axis than with its diagonal
  double diff = std::abs(std::remainder(grasp_yaw - object_yaw, 2.0 * M_PI));
  diff = diff > M_PI_2 ? M_PI - diff : diff;
  double diagonal_angle = std::atan(object_size.y / object_size.x);
  bool x_aligned = diff + diagonal_angle < M_PI_2;
  return (x_aligned ? object_size.y : object_size.x) - tightening;
}

}  // namespace thorp::manipulation
