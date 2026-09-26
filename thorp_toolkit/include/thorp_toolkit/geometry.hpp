/*
 * Author: Jorge Santos
 */

#pragma once

#include <array>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "thorp_toolkit/common.hpp"

namespace thorp::toolkit
{

/**
 * Convert angle from radians to degrees
 * @param a Angle in radians
 * @return Angle in degrees
 */
inline double toDeg(double a)
{
  return angles::to_degrees(a);
}

/**
 * Convert angle from degrees to radians
 * @param a Angle in degrees
 * @return Angle in radians
 */
inline double toRad(double a)
{
  return angles::from_degrees(a);
}

/**
 * Normalize an angle between -π and +π
 * @param a Unnormalized angle
 * @return Normalized angle
 */
inline double normAngle(double a)
{
  a = fmod(a + M_PI, 2*M_PI);
  return a < 0.0 ? a + M_PI : a - M_PI;
}

/**
 * Shortcut to take the roll of a transform
 * @param tf the transform
 * @return transform's roll
 */
inline double roll(const tf2::Transform& tf)
{
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);
  return roll;
}

/**
 * Shortcut to take the roll of a pose
 * @param the pose
 * @return pose's roll
 */
inline double roll(const geometry_msgs::msg::Pose& pose)
{
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return roll;
}

/**
 * Shortcut to take the roll of a stamped pose
 * @param pose the pose
 * @return pose roll
 */
inline double roll(const geometry_msgs::msg::PoseStamped& pose)
{
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return roll;
}

/**
 * Shortcut to take the roll of a stamped transform
 * @param tf the transform
 * @return transform's roll
 */
inline double roll(const geometry_msgs::msg::TransformStamped& tf)
{
  tf2::Quaternion q;
  tf2::fromMsg(tf.transform.rotation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return roll;
}

/**
 * Shortcut to take the pitch of a transform
 * @param tf the transform
 * @return transform's pitch
 */
inline double pitch(const tf2::Transform& tf)
{
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);
  return pitch;
}

/**
 * Shortcut to take the pitch of a pose
 * @param the pose
 * @return pose's pitch
 */
inline double pitch(const geometry_msgs::msg::Pose& pose)
{
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return pitch;
}

/**
 * Shortcut to take the pitch of a stamped pose
 * @param the pose
 * @return pose's pitch
 */
inline double pitch(const geometry_msgs::msg::PoseStamped& pose)
{
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return pitch;
}

/**
 * Shortcut to take the pitch of a stamped transform
 * @param tf the transform
 * @return transform's pitch
 */
inline double pitch(const geometry_msgs::msg::TransformStamped& tf)
{
  tf2::Quaternion q;
  tf2::fromMsg(tf.transform.rotation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return pitch;
}

/**
 * Shortcut to take the yaw of a transform
 * @param tf the transform
 * @return transform's yaw
 */
inline double yaw(const tf2::Transform& tf)
{
  return tf2::getYaw(tf.getRotation());
}

/**
 * Shortcut to take the yaw of a quaternion
 * @param q quaternion q
 * @return quaternion's yaw
 */
inline double yaw(const tf2::Quaternion& q)
{
  return tf2::getYaw(q);
}

/**
 * Alias of tf2::getYaw(quaternion)
 * @param q quaternion q
 * @return quaternion's yaw
 */
inline double yaw(const geometry_msgs::msg::Quaternion& q)
{
  return tf2::getYaw(q);
}

/**
 * Alias of tf2::getYaw(quaternion)
 * @param pose the pose
 * @return pose's yaw
 */
inline double yaw(const geometry_msgs::msg::Pose& pose)
{
  return tf2::getYaw(pose.orientation);
}

/**
 * Shortcut to take the yaw of a stamped pose
 * @param pose the pose
 * @return pose's yaw
 */
inline double yaw(const geometry_msgs::msg::PoseStamped& pose)
{
  return tf2::getYaw(pose.pose.orientation);
}

/**
 * Shortcut to take the yaw of a stamped transform
 * @param tf the transform
 * @return transform's yaw
 */
inline double yaw(const geometry_msgs::msg::TransformStamped& tf)
{
  return tf2::getYaw(tf.transform.rotation);
}

/**
 * Set the yaw of a given pose.
 * @warning This is only intended for 2D poses, as it will discard pitch and roll.
 * @param pose Pose
 */
inline void setYaw(geometry_msgs::msg::Pose& pose, double new_yaw)
{
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0, 0, new_yaw);
  pose.orientation = tf2::toMsg(quat_tf);
}

/**
 * Set the yaw of a given pose.
 * @warning This is only intended for 2D poses, as it will discard pitch and roll.
 * @param pose Stamped pose
 */
inline void setYaw(geometry_msgs::msg::PoseStamped& pose, double new_yaw)
{
  setYaw(pose.pose, new_yaw);
}

/**
 * Check if a 2D pose is pointing more along x or y axis
 * @param pose the pose
 * @return true if pointing more along x axis, false if along y
 */
inline bool xAligned(const geometry_msgs::msg::Pose& pose)
{
  double abs_yaw = std::abs(tf2::getYaw(pose.orientation));
  return abs_yaw < M_PI/4.0 || abs_yaw > M_PI*3.0/4.0;
}

/**
 * Compares frame ids ignoring the leading /, as it is frequently omitted.
 * @param frame_a
 * @param frame_b
 * @return true if frame ids match regardless leading /
 */
inline bool sameFrame(const std::string& frame_a, const std::string& frame_b)
{
  if (frame_a.length() == 0 && frame_b.length() == 0)
  {
    RCLCPP_WARN(logger(), "Comparing two empty frame ids (considered as the same frame)");
    return true;
  }

  if (frame_a.length() == 0 || frame_b.length() == 0)
  {
    RCLCPP_WARN(logger(), "Comparing %s%s with an empty frame id (considered as different frames)",
             frame_a.c_str(), frame_b.c_str());
    return false;
  }

  int start_a = frame_a.at(0) == '/' ? 1 : 0;
  int start_b = frame_b.at(0) == '/' ? 1 : 0;

  return frame_a.compare(start_a, frame_a.length(), frame_b, start_b, frame_b.length()) == 0;
}

/**
 * Compares if two stamped poses have the same frame id, ignoring the leading /, as it is frequently omitted.
 * @param a pose a
 * @param b pose b
 * @return true if poses' frame ids match regardless leading /
 */
inline bool sameFrame(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b)
{
  return sameFrame(a.header.frame_id, b.header.frame_id);
}

/**
 * Compares if two stamped poses have the same frame id, ignoring the leading /, as it is frequently omitted.
 * @param a pose a
 * @param b pose b
 * @return true if poses' frame ids match regardless leading /
 */
inline bool sameFrame(const tf2::Stamped<tf2::Transform>& a, const tf2::Stamped<tf2::Transform>& b)
{
  return sameFrame(a.frame_id_, b.frame_id_);
}


/**
 * Euclidean distance between 2D points or one point and the origin; z coordinate is ignored.
 * @param a point a
 * @param b point b
 * @return distance
 */
inline double distance2D(double ax, double ay, double bx = 0.0, double by = 0.0)
{
  return std::sqrt(std::pow(ax - bx, 2) + std::pow(ay - by, 2));
}

/**
 * Euclidean distance between 2D points or one point and the origin; z coordinate is ignored.
 * @param a point a
 * @param b point b
 * @return distance
 */
inline double distance2D(const tf2::Vector3& a, const tf2::Vector3& b = tf2::Vector3(0.0, 0.0, 0.0))
{
  return std::sqrt(std::pow(b.x() - a.x(), 2) + std::pow(b.y() - a.y(), 2));
}

/**
 * Euclidean distance between 2D points or one point and the origin; z coordinate is ignored.
 * @param a point a
 * @param b point b
 * @return distance
 */
inline double distance2D(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b = geometry_msgs::msg::Point())
{
  return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

/**
 * Euclidean distance between 2D poses or one pose and the origin; z coordinate is ignored.
 * @param a pose a
 * @param b pose b
 * @return distance
 */
inline double distance2D(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b = geometry_msgs::msg::Pose())
{
  return std::sqrt(std::pow(b.position.x - a.position.x, 2) + std::pow(b.position.y - a.position.y, 2));
}

/**
 * Euclidean distance between 2D stamped poses or one pose and the origin; z coordinate is ignored.
 * Assumes both poses are in the same reference frame.
 * @param a pose a
 * @param b pose b
 * @return distance
 */
inline double distance2D(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b = geometry_msgs::msg::PoseStamped())
{
  return std::sqrt(std::pow(b.pose.position.x - a.pose.position.x, 2)
                 + std::pow(b.pose.position.y - a.pose.position.y, 2));
}

/**
 * Euclidean distance between 2D stamped poses or one pose and the origin; z coordinate is ignored.
 * Assumes both poses are in the same reference frame.
 * @param a pose a
 * @param b pose b
 * @return distance
 */
inline double distance2D(const tf2::Stamped<tf2::Transform>& a, const tf2::Stamped<tf2::Transform>& b = tf2::Stamped<tf2::Transform>(tf2::Transform::getIdentity(), tf2::TimePointZero, ""))
{
  return std::sqrt(std::pow(b.getOrigin().x() - a.getOrigin().x(), 2)
                 + std::pow(b.getOrigin().y() - a.getOrigin().y(), 2));
}

/**
 * Euclidean distance between 2D transforms or one transform and the origin; z coordinate is ignored.
 * @param a transform a
 * @param b transform b
 * @return distance
 */
inline double distance2D(const tf2::Transform& a, const tf2::Transform& b = tf2::Transform::getIdentity())
{
  return std::sqrt(std::pow(b.getOrigin().x() - a.getOrigin().x(), 2)
                 + std::pow(b.getOrigin().y() - a.getOrigin().y(), 2));
}

/**
 * Computes the dot product between the two given vectors
 * @param v1 Vector AB {from A, to B}
 * @param v2 Vector CD {from C, to D}
 * @return AB.CD
 */
double dot(const std::array<geometry_msgs::msg::Point, 2>& v1, const std::array<geometry_msgs::msg::Point, 2>& v2);

/**
 * Euclidean distance between a point and a line segment.
 * @param line start and end points on the line segment
 * @param point point to which distance needs to be calculated from
 * @return distance from point to line segment
 */
double distance2D(const std::array<geometry_msgs::msg::Point, 2>& line, const geometry_msgs::msg::Point& point);


/**
 * Euclidean distance between 3D points or one point and the origin.
 * @param a point a
 * @param b point b
 * @return distance
 */
inline double distance3D(double ax, double ay, double az, double bx = 0.0, double by = 0.0, double bz = 0.0)
{
  return std::sqrt(std::pow(ax - bx, 2) + std::pow(ay - by, 2) + std::pow(az - bz, 2));
}

/**
 * Euclidean distance between 3D points or one point and the origin.
 * @param a point a
 * @param b point b
 * @return distance
 */
inline double distance3D(const tf2::Vector3& a, const tf2::Vector3& b = tf2::Vector3(0.0, 0.0, 0.0))
{
  return std::sqrt(std::pow(b.x() - a.x(), 2) + std::pow(b.y() - a.y(), 2) + std::pow(b.z() - a.z(), 2));
}

/**
 * Euclidean distance between 3D points or one point and the origin.
 * @param a point a
 * @param b point b
 * @return distance
 */
inline double distance3D(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b = geometry_msgs::msg::Point())
{
  return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2) + std::pow(b.z - a.z, 2));
}

/**
 * Euclidean distance between 3D poses or one pose and the origin.
 * @param a pose a
 * @param b pose b
 * @return distance
 */
inline double distance3D(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b = geometry_msgs::msg::Pose())
{
  return std::sqrt(std::pow(b.position.x - a.position.x, 2)
                 + std::pow(b.position.y - a.position.y, 2)
                 + std::pow(b.position.z - a.position.z, 2));
}

/**
 * Euclidean distance between 3D stamped poses or one pose and the origin.
 * Assumes both poses are in the same reference frame.
 * @param a pose a
 * @param b pose b
 * @return distance
 */
inline double distance3D(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b = geometry_msgs::msg::PoseStamped())
{
  return std::sqrt(std::pow(b.pose.position.x - a.pose.position.x, 2)
                 + std::pow(b.pose.position.y - a.pose.position.y, 2)
                 + std::pow(b.pose.position.z - a.pose.position.z, 2));
}

/**
 * Euclidean distance between 3D stamped poses or one pose and the origin.
 * Assumes both poses are in the same reference frame.
 * @param a pose a
 * @param b pose b
 * @return distance
 */
inline double distance3D(const tf2::Stamped<tf2::Transform>& a, const tf2::Stamped<tf2::Transform>& b = tf2::Stamped<tf2::Transform>(tf2::Transform::getIdentity(), tf2::TimePointZero, ""))
{
  return std::sqrt(std::pow(b.getOrigin().x() - a.getOrigin().x(), 2)
                 + std::pow(b.getOrigin().y() - a.getOrigin().y(), 2)
                 + std::pow(b.getOrigin().z() - a.getOrigin().z(), 2));
}

/**
 * Euclidean distance between 3D transforms or one transform and the origin.
 * @param a transform a
 * @param b transform b
 * @return distance
 */
inline double distance3D(const tf2::Transform& a, const tf2::Transform& b = tf2::Transform::getIdentity())
{
  return std::sqrt(std::pow(b.getOrigin().x() - a.getOrigin().x(), 2)
                 + std::pow(b.getOrigin().y() - a.getOrigin().y(), 2)
                 + std::pow(b.getOrigin().z() - a.getOrigin().z(), 2));
}

/**
 * Heading angle from origin to point p
 * @param p the point
 * @return heading angle
 */
inline double heading(const tf2::Vector3& p)
{
  return std::atan2(p.y(), p.x());
}

/**
 * Heading angle from origin to point p
 * @param p the point
 * @return heading angle
 */
inline double heading(const geometry_msgs::msg::Point& p)
{
  return std::atan2(p.y, p.x);
}

/**
 * Heading angle from origin to pose p
 * @param p the pose
 * @return heading angle
 */
inline double heading(const geometry_msgs::msg::Pose& p)
{
  return std::atan2(p.position.y, p.position.x);
}

/**
 * Heading angle from origin to stamped pose p
 * @param p the pose
 * @return heading angle
 */
inline double heading(const geometry_msgs::msg::PoseStamped& p)
{
  return std::atan2(p.pose.position.y, p.pose.position.x);
}

/**
 * Heading angle from origin to transform p
 * @param t the transform
 * @return heading angle
 */
inline double heading(const tf2::Transform& t)
{
  return std::atan2(t.getOrigin().y(), t.getOrigin().x());
}


/**
 * Heading angle from point a to point b. Returns NaN if both points are the same.
 * Note: atan2(0, 0) returns 0.
 * @param a point a
 * @param b point b
 * @return heading angle
 */
inline double heading(const tf2::Vector3& a, const tf2::Vector3& b)
{
  if (b.y() - a.y() == 0.0 && b.x() - a.x() == 0.0)
    return NAN;

  return std::atan2(b.y() - a.y(), b.x() - a.x());
}

/**
 * Heading angle from point a to point b. Returns NaN if both points are the same.
 * Note: atan2(0, 0) returns 0.
 * @param a point a
 * @param b point b
 * @return heading angle
 */
inline double heading(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
  if (b.y - a.y == 0.0 && b.x - a.x == 0.0)
    return NAN;

  return std::atan2(b.y - a.y, b.x - a.x);
}

/**
 * Heading angle from pose a to pose b. Returns NaN if both points are the same.
 * Note: atan2(0, 0) returns 0.
 * @param a pose a
 * @param b pose b
 * @return heading angle
 */
inline double heading(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b)
{
  if (b.position.y - a.position.y == 0.0 && b.position.x - a.position.x == 0.0)
    return NAN;

  return std::atan2(b.position.y - a.position.y, b.position.x - a.position.x);
}

/**
 * Heading angle from stamped pose a to stamped pose b. Returns NaN if both poses have the same 2D position.
 * Assumes both poses are in the same reference frame. Note: atan2(0, 0) returns 0.
 * @param a pose a
 * @param b pose b
 * @return heading angle
 */
inline double heading(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b)
{
  if (b.pose.position.y - a.pose.position.y == 0.0 && b.pose.position.x - a.pose.position.x == 0.0)
    return NAN;

  return std::atan2(b.pose.position.y - a.pose.position.y, b.pose.position.x - a.pose.position.x);
}

/**
 * Heading angle from transform a to transform b. Returns NaN if both transforms have the same 2D origin.
 * Note: atan2(0, 0) returns 0.
 * @param a transform a
 * @param b transform b
 * @return heading angle
 */
inline double heading(const tf2::Transform& a, const tf2::Transform& b)
{
  if (b.getOrigin().y() - a.getOrigin().y() == 0.0 && b.getOrigin().x() - a.getOrigin().x() == 0.0)
    return NAN;

  return std::atan2(b.getOrigin().y() - a.getOrigin().y(), b.getOrigin().x() - a.getOrigin().x());
}

/**
 * Minimum angle between quaternions
 * @param a quaternion a
 * @param b quaternion b
 * @return minimum angle
 */
inline double minAngle(const tf2::Quaternion& a, const tf2::Quaternion& b)
{
  return angles::shortest_angular_distance(yaw(a), yaw(b));
}

/**
 * Minimum angle between quaternions
 * @param a quaternion a
 * @param b quaternion b
 * @return minimum angle
 */
inline double minAngle(const geometry_msgs::msg::Quaternion& a, const geometry_msgs::msg::Quaternion& b)
{
  return angles::shortest_angular_distance(yaw(a), yaw(b));
}

/**
 * Minimum angle between pose orientations
 * @param a pose a
 * @param b pose b
 * @return minimum angle
 */
inline double minAngle(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b)
{
  return angles::shortest_angular_distance(yaw(a), yaw(b));
}

/**
 * Minimum angle between pose orientations. Assumes both poses are in the same reference frame.
 * @param a pose a
 * @param b pose b
 * @return minimum angle
 */
inline double minAngle(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b)
{
  return angles::shortest_angular_distance(yaw(a), yaw(b));
}

/**
 * Minimum angle between transform rotations
 * @param a transform a
 * @param b transform b
 * @return minimum angle
 */
inline double minAngle(const tf2::Transform& a, const tf2::Transform& b)
{
  return angles::shortest_angular_distance(yaw(a), yaw(b));
}

/**
 * Shortest angular difference between two angles.
 * Just a wrapper around shortest_angular_distance to avoid depending on angles package.
 * @param a angle a
 * @param b angle b
 * @return shortest angular difference
 */
inline double anglesDiff(double a, double b)
{
  return angles::shortest_angular_distance(a, b);
}


/**
 * Compares two poses to be (nearly) the same within tolerance margins.
 * @param a pose a
 * @param b pose b
 * @param xy_tolerance linear distance tolerance
 * @param yaw_tolerance angular distance tolerance
 * @return true if both poses are the same within tolerance margins.
 */
inline bool samePose(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b,
                     double xy_tolerance = 0.0001, double yaw_tolerance = 0.0001)
{
  return distance2D(a, b) <= xy_tolerance && std::abs(minAngle(a, b)) <= yaw_tolerance;
}

/**
 * Compares two poses to be (nearly) the same within tolerance margins.
 * @param a pose a
 * @param b pose b
 * @param xy_tolerance linear distance tolerance
 * @param yaw_tolerance angular distance tolerance
 * @return true if both poses are the same within tolerance margins.
 */
inline bool samePose(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b,
                     double xy_tolerance = 0.0001, double yaw_tolerance = 0.0001)
{
  return sameFrame(a, b) && samePose(a.pose, b.pose, xy_tolerance, yaw_tolerance);
}

geometry_msgs::msg::Pose createPose(double x, double y, double yaw);

geometry_msgs::msg::Pose createPose(double x, double y, double z, double roll, double pitch, double yaw);

geometry_msgs::msg::PoseStamped createPose(double x, double y, double yaw, const std::string& frame);

geometry_msgs::msg::PoseStamped createPose(double x, double y, double z, double roll, double pitch, double yaw,
                                      const std::string& frame);

geometry_msgs::msg::Transform toTransform(const geometry_msgs::msg::Pose& pose);

geometry_msgs::msg::TransformStamped toTransform(const geometry_msgs::msg::PoseStamped& pose);

geometry_msgs::msg::PoseArray toPoseArray(const std::vector<geometry_msgs::msg::PoseStamped>& poses, double z_delta = 0.0,
                                     const std::string& default_frame_id = "map");

/**
 * Extract a 2D pose from a geometry_msgs/Pose msg.
 * @param pose 3D pose msg.
 * @return 2D pose msg.
 */
geometry_msgs::msg::Pose2D toPose2D(const geometry_msgs::msg::Pose& pose);
geometry_msgs::msg::Pose toPose3D(const geometry_msgs::msg::Pose2D& pose);
geometry_msgs::msg::Point32 toPoint(const geometry_msgs::msg::Point& point);
geometry_msgs::msg::Point toPoint(const geometry_msgs::msg::Point32& point);


std::string toStr3D(const geometry_msgs::msg::Vector3& vector);
std::string toStr3D(const geometry_msgs::msg::Vector3Stamped& vector);
std::string toStr2D(const geometry_msgs::msg::Point& point);
std::string toStr2D(const geometry_msgs::msg::PointStamped& point);
std::string toStr3D(const geometry_msgs::msg::Point& point);
std::string toStr3D(const geometry_msgs::msg::PointStamped& point);
std::string toStr2D(const geometry_msgs::msg::Pose& pose);
std::string toStr2D(const geometry_msgs::msg::PoseStamped& pose);
std::string toStr2D(const tf2::Stamped<tf2::Transform>& pose);
std::string toStr3D(const geometry_msgs::msg::Pose& pose);
std::string toStr3D(const geometry_msgs::msg::PoseStamped& pose);
std::string toStr3D(const tf2::Stamped<tf2::Transform>& pose);

const char* toCStr3D(const geometry_msgs::msg::Vector3& vector);
const char* toCStr3D(const geometry_msgs::msg::Vector3Stamped& vector);
const char* toCStr2D(const geometry_msgs::msg::Point& point);
const char* toCStr2D(const geometry_msgs::msg::PointStamped& point);
const char* toCStr3D(const geometry_msgs::msg::Point& point);
const char* toCStr3D(const geometry_msgs::msg::PointStamped& point);
const char* toCStr2D(const geometry_msgs::msg::Pose& pose);
const char* toCStr2D(const geometry_msgs::msg::PoseStamped& pose);
const char* toCStr2D(const tf2::Stamped<tf2::Transform>& pose);
const char* toCStr3D(const geometry_msgs::msg::Pose& pose);
const char* toCStr3D(const geometry_msgs::msg::PoseStamped& pose);
const char* toCStr3D(const tf2::Stamped<tf2::Transform>& pose);

/**
 * Clip line segment to fit within a bounding box using Liang-Barsky function by Daniel White
 * @ http://www.skytopia.com/project/articles/compsci/clipping.html
 * This function inputs 8 numbers, and outputs 4 new numbers (plus a boolean value to say whether
 * the clipped line exists, or the segment laid completely outside of the clipping bounding box).
 * @param edge_left Clipping box left edge.
 * @param edge_right Clipping box right edge.
 * @param edge_bottom Clipping box bottom edge.
 * @param edge_top Clipping box top edge.
 * @param x0src Line segment start point x-coordinate
 * @param y0src Line segment start point y-coordinate
 * @param x1src Line segment end point x-coordinate
 * @param y1src Line segment end point y-coordinate
 * @param x0clip Clipped segment start point x-coordinate
 * @param y0clip Clipped segment start point y-coordinate
 * @param x1clip Clipped segment end point x-coordinate
 * @param y1clip Clipped segment end point y-coordinate
 * @return True if the clipped line exists
 */
bool clipSegment(double edge_left, double edge_right, double edge_bottom, double edge_top,
                 double x0src, double y0src, double x1src, double y1src,
                 double& x0clip, double& y0clip, double& x1clip, double& y1clip);

/**
 * Area of the triangle formed by three poses (see https://www.mathopenref.com/coordtrianglearea.html)
 * @param pose_a
 * @param pose_b
 * @param pose_c
 * @return area
 */
double enclosedArea(const geometry_msgs::msg::PoseStamped& pose_a,
                    const geometry_msgs::msg::PoseStamped& pose_b,
                    const geometry_msgs::msg::PoseStamped& pose_c);

/**
 * Menger curvature using triple poses (see https://en.wikipedia.org/wiki/Menger_curvature)
 * @param pose_a
 * @param pose_b
 * @param pose_c
 * @return curvature
 */
double curvature(const geometry_msgs::msg::PoseStamped& pose_a,
                 const geometry_msgs::msg::PoseStamped& pose_b,
                 const geometry_msgs::msg::PoseStamped& pose_c);

} /* namespace thorp::toolkit */
