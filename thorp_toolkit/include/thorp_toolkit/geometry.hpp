/*
 * Author: Jorge Santos
 */

#pragma once

#include <numeric>

#include <tf/tf.h>
#include <angles/angles.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


namespace thorp_toolkit
{

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
inline double roll(const tf::Transform& tf)
{
  double roll, pitch, yaw;
  tf::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);
  return roll;
}

/**
 * Shortcut to take the roll of a pose
 * @param the pose
 * @return pose's roll
 */
inline double roll(const geometry_msgs::Pose& pose)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return roll;
}

/**
 * Shortcut to take the roll of a stamped pose
 * @param pose the pose
 * @return pose roll
 */
inline double roll(const geometry_msgs::PoseStamped& pose)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return roll;
}

/**
 * Shortcut to take the roll of a stamped transform
 * @param tf the transform
 * @return transform's roll
 */
inline double roll(const geometry_msgs::TransformStamped& tf)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(tf.transform.rotation, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return roll;
}

/**
 * Shortcut to take the pitch of a transform
 * @param tf the transform
 * @return transform's pitch
 */
inline double pitch(const tf::Transform& tf)
{
  double roll, pitch, yaw;
  tf::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);
  return pitch;
}

/**
 * Shortcut to take the pitch of a pose
 * @param the pose
 * @return pose's pitch
 */
inline double pitch(const geometry_msgs::Pose& pose)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return pitch;
}

/**
 * Shortcut to take the pitch of a stamped pose
 * @param the pose
 * @return pose's pitch
 */
inline double pitch(const geometry_msgs::PoseStamped& pose)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return pitch;
}

/**
 * Shortcut to take the pitch of a stamped transform
 * @param tf the transform
 * @return transform's pitch
 */
inline double pitch(const geometry_msgs::TransformStamped& tf)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(tf.transform.rotation, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return pitch;
}

/**
 * Shortcut to take the yaw of a transform
 * @param tf the transform
 * @return transform's yaw
 */
inline double yaw(const tf::Transform& tf)
{
  return tf::getYaw(tf.getRotation());
}

/**
 * Shortcut to take the yaw of a quaternion
 * @param q quaternion q
 * @return quaternion's yaw
 */
inline double yaw(const tf::Quaternion& q)
{
  return tf::getYaw(q);
}

/**
 * Alias of tf::getYaw(quaternion)
 * @param q quaternion q
 * @return quaternion's yaw
 */
inline double yaw(const geometry_msgs::Quaternion& q)
{
  return tf::getYaw(q);
}

/**
 * Alias of tf::getYaw(quaternion)
 * @param pose the pose
 * @return pose's yaw
 */
inline double yaw(const geometry_msgs::Pose& pose)
{
  return tf::getYaw(pose.orientation);
}

/**
 * Shortcut to take the yaw of a stamped pose
 * @param pose the pose
 * @return pose's yaw
 */
inline double yaw(const geometry_msgs::PoseStamped& pose)
{
  return tf::getYaw(pose.pose.orientation);
}

/**
 * Shortcut to take the yaw of a stamped transform
 * @param tf the transform
 * @return transform's yaw
 */
inline double yaw(const geometry_msgs::TransformStamped& tf)
{
  return tf::getYaw(tf.transform.rotation);
}

/**
 * Check if a 2D pose is pointing more along x or y axis
 * @param pose the pose
 * @return true if pointing more along x axis, false if along y
 */
inline bool xAligned(const geometry_msgs::Pose& pose)
{
  double abs_yaw = std::abs(tf::getYaw(pose.orientation));
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
    ROS_WARN("Comparing two empty frame ids (considered as the same frame)");
    return true;
  }

  if (frame_a.length() == 0 || frame_b.length() == 0)
  {
    ROS_WARN("Comparing %s%s with an empty frame id (considered as different frames)",
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
inline bool sameFrame(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b)
{
  return sameFrame(a.header.frame_id, b.header.frame_id);
}

/**
 * Compares if two stamped poses have the same frame id, ignoring the leading /, as it is frequently omitted.
 * @param a pose a
 * @param b pose b
 * @return true if poses' frame ids match regardless leading /
 */
inline bool sameFrame(const tf::Stamped<tf::Pose>& a, const tf::Stamped<tf::Pose>& b)
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
inline double distance2D(const tf::Point& a, const tf::Point& b = tf::Point())
{
  return std::sqrt(std::pow(b.x() - a.x(), 2) + std::pow(b.y() - a.y(), 2));
}

/**
 * Euclidean distance between 2D points or one point and the origin; z coordinate is ignored.
 * @param a point a
 * @param b point b
 * @return distance
 */
inline double distance2D(const geometry_msgs::Point& a, const geometry_msgs::Point& b = geometry_msgs::Point())
{
  return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

/**
 * Euclidean distance between 2D poses or one pose and the origin; z coordinate is ignored.
 * @param a pose a
 * @param b pose b
 * @return distance
 */
inline double distance2D(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b = geometry_msgs::Pose())
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
inline double distance2D(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b = geometry_msgs::PoseStamped())
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
inline double distance2D(const tf::Stamped<tf::Pose>& a, const tf::Stamped<tf::Pose>& b = tf::Stamped<tf::Pose>())
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
inline double distance2D(const tf::Transform& a, const tf::Transform& b = tf::Transform())
{
  return std::sqrt(std::pow(b.getOrigin().x() - a.getOrigin().x(), 2)
                 + std::pow(b.getOrigin().y() - a.getOrigin().y(), 2));
}


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
inline double distance3D(const tf::Point& a, const tf::Point& b = tf::Point())
{
  return std::sqrt(std::pow(b.x() - a.x(), 2) + std::pow(b.y() - a.y(), 2) + std::pow(b.z() - a.z(), 2));
}

/**
 * Euclidean distance between 3D points or one point and the origin.
 * @param a point a
 * @param b point b
 * @return distance
 */
inline double distance3D(const geometry_msgs::Point& a, const geometry_msgs::Point& b = geometry_msgs::Point())
{
  return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2) + std::pow(b.z - a.z, 2));
}

/**
 * Euclidean distance between 3D poses or one pose and the origin.
 * @param a pose a
 * @param b pose b
 * @return distance
 */
inline double distance3D(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b = geometry_msgs::Pose())
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
inline double distance3D(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b = geometry_msgs::PoseStamped())
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
inline double distance3D(const tf::Stamped<tf::Pose>& a, const tf::Stamped<tf::Pose>& b = tf::Stamped<tf::Pose>())
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
inline double distance3D(const tf::Transform& a, const tf::Transform& b = tf::Transform())
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
inline double heading(const tf::Vector3& p)
{
  return std::atan2(p.y(), p.x());
}

/**
 * Heading angle from origin to point p
 * @param p the point
 * @return heading angle
 */
inline double heading(const geometry_msgs::Point& p)
{
  return std::atan2(p.y, p.x);
}

/**
 * Heading angle from origin to pose p
 * @param p the pose
 * @return heading angle
 */
inline double heading(const geometry_msgs::Pose& p)
{
  return std::atan2(p.position.y, p.position.x);
}

/**
 * Heading angle from origin to stamped pose p
 * @param p the pose
 * @return heading angle
 */
inline double heading(const geometry_msgs::PoseStamped& p)
{
  return std::atan2(p.pose.position.y, p.pose.position.x);
}

/**
 * Heading angle from origin to transform p
 * @param t the transform
 * @return heading angle
 */
inline double heading(const tf::Transform& t)
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
inline double heading(const tf::Vector3& a, const tf::Vector3& b)
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
inline double heading(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
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
inline double heading(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
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
inline double heading(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b)
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
inline double heading(const tf::Transform& a, const tf::Transform& b)
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
inline double minAngle(const tf::Quaternion& a, const tf::Quaternion& b)
{
  return angles::shortest_angular_distance(yaw(a), yaw(b));
}

/**
 * Minimum angle between quaternions
 * @param a quaternion a
 * @param b quaternion b
 * @return minimum angle
 */
inline double minAngle(const geometry_msgs::Quaternion& a, const geometry_msgs::Quaternion& b)
{
  return angles::shortest_angular_distance(yaw(a), yaw(b));
}

/**
 * Minimum angle between pose orientations
 * @param a pose a
 * @param b pose b
 * @return minimum angle
 */
inline double minAngle(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
  return angles::shortest_angular_distance(yaw(a), yaw(b));
}

/**
 * Minimum angle between pose orientations. Assumes both poses are in the same reference frame.
 * @param a pose a
 * @param b pose b
 * @return minimum angle
 */
inline double minAngle(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b)
{
  return angles::shortest_angular_distance(yaw(a), yaw(b));
}

/**
 * Minimum angle between transform rotations
 * @param a transform a
 * @param b transform b
 * @return minimum angle
 */
inline double minAngle(const tf::Transform& a, const tf::Transform& b)
{
  return angles::shortest_angular_distance(yaw(a), yaw(b));
}


/**
 * Compares two poses to be (nearly) the same within tolerance margins.
 * @param a pose a
 * @param b pose b
 * @param xy_tolerance linear distance tolerance
 * @param yaw_tolerance angular distance tolerance
 * @return true if both poses are the same within tolerance margins.
 */
inline bool samePose(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b,
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
inline bool samePose(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b,
                     double xy_tolerance = 0.0001, double yaw_tolerance = 0.0001)
{
  return sameFrame(a, b) && samePose(a.pose, b.pose, xy_tolerance, yaw_tolerance);
}

std::string vector2str3D(const geometry_msgs::Vector3& vector);
std::string vector2str3D(const geometry_msgs::Vector3Stamped& vector);
std::string point2str2D(const geometry_msgs::Point& point);
std::string point2str2D(const geometry_msgs::PointStamped& point);
std::string point2str3D(const geometry_msgs::Point& point);
std::string point2str3D(const geometry_msgs::PointStamped& point);
std::string pose2str2D(const geometry_msgs::Pose& pose);
std::string pose2str2D(const geometry_msgs::PoseStamped& pose);
std::string pose2str2D(const tf::Stamped<tf::Pose>& pose);
std::string pose2str3D(const geometry_msgs::Pose& pose);
std::string pose2str3D(const geometry_msgs::PoseStamped& pose);
std::string pose2str3D(const tf::Stamped<tf::Pose>& pose);

const char* vector2cstr3D(const geometry_msgs::Vector3& vector);
const char* vector2cstr3D(const geometry_msgs::Vector3Stamped& vector);
const char* point2cstr2D(const geometry_msgs::Point& point);
const char* point2cstr2D(const geometry_msgs::PointStamped& point);
const char* point2cstr3D(const geometry_msgs::Point& point);
const char* point2cstr3D(const geometry_msgs::PointStamped& point);
const char* pose2cstr2D(const geometry_msgs::Pose& pose);
const char* pose2cstr2D(const geometry_msgs::PoseStamped& pose);
const char* pose2cstr2D(const tf::Stamped<tf::Pose>& pose);
const char* pose2cstr3D(const geometry_msgs::Pose& pose);
const char* pose2cstr3D(const geometry_msgs::PoseStamped& pose);
const char* pose2cstr3D(const tf::Stamped<tf::Pose>& pose);

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
double enclosedArea(const geometry_msgs::PoseStamped& pose_a,
                    const geometry_msgs::PoseStamped& pose_b,
                    const geometry_msgs::PoseStamped& pose_c);

/**
 * Menger curvature using triple poses (see https://en.wikipedia.org/wiki/Menger_curvature)
 * @param pose_a
 * @param pose_b
 * @param pose_c
 * @return curvature
 */
double curvature(const geometry_msgs::PoseStamped& pose_a,
                 const geometry_msgs::PoseStamped& pose_b,
                 const geometry_msgs::PoseStamped& pose_c);

} /* namespace thorp_toolkit */
