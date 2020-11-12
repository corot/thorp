/*
 * geometry.hpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#pragma once


#include <tf/tf.h>
#include <numeric>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


namespace mag_common_libs
{

/**
 * Normalize an angle between -π and +π
 * @param a Unnormalized angle
 * @return Normalized angle
 */
inline double wrapAngle(double a)
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
 * Shortcut to take the yaw of a transform
 * @param tf the transform
 * @return transform's yaw
 */
inline double yaw(const tf::Transform& tf)
{
  return tf::getYaw(tf.getRotation());
}

/**
 * Shortcut to take the yaw of a pose
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
                 + std::pow(b.getOrigin().z() - a.getOrigin().z(), 2));;
}


/**
 * Euclidean distance between a pose and the closest pose to it in a trajectory; z coordinate is ignored.
 * Provides also the closest pose's index in the trajectory.
 * @param p pose
 * @param t trajectory
 * @param closest_pose closest pose index
 * @return Distance
 */
inline double distance2D(const geometry_msgs::PoseStamped& p,
                         const std::vector<geometry_msgs::PoseStamped>& t, size_t& closest_pose)
{
  double min_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < t.size(); i++)
  {
    double distance = distance2D(p, t[i]);
    if (distance < min_distance)
    {
      min_distance = distance;
      closest_pose = i;
    }
  }
  return min_distance;
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
  return tf::angleShortestPath(a, b);
}

/**
 * Minimum angle between quaternions
 * @param a quaternion a
 * @param b quaternion b
 * @return minimum angle
 */
inline double minAngle(const geometry_msgs::Quaternion& a, const geometry_msgs::Quaternion& b)
{
  tf::Quaternion qa(a.x, a.y, a.z, a.w);
  tf::Quaternion qb(b.x, b.y, b.z, b.w);
  return tf::angleShortestPath(qa, qb);
}

/**
 * Minimum angle between pose orientations
 * @param a pose a
 * @param b pose b
 * @return minimum angle
 */
inline double minAngle(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
  tf::Quaternion qa(a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w);
  tf::Quaternion qb(b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w);
  return tf::angleShortestPath(qa, qb);
}

/**
 * Minimum angle between pose orientations. Assumes both poses are in the same reference frame.
 * @param a pose a
 * @param b pose b
 * @return minimum angle
 */
inline double minAngle(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b)
{
  tf::Quaternion qa(a.pose.orientation.x, a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w);
  tf::Quaternion qb(b.pose.orientation.x, b.pose.orientation.y, b.pose.orientation.z, b.pose.orientation.w);
  return tf::angleShortestPath(qa, qb);
}

/**
 * Minimum angle between transform rotations
 * @param a transform a
 * @param b transform b
 * @return minimum angle
 */
inline double minAngle(const tf::Transform& a, const tf::Transform& b)
{
  tf::Quaternion qa(a.getRotation().x(), a.getRotation().y(), a.getRotation().z(), a.getRotation().w());
  tf::Quaternion qb(b.getRotation().x(), b.getRotation().y(), b.getRotation().z(), b.getRotation().w());
  return tf::angleShortestPath(qa, qb);
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

/**
 * Transform in_pose from on frame to another
 * @param from_frame initial frame
 * @param to_frame target frame
 * @param in_pose initial pose
 * @param out_pose transformed pose
 * @return true if transformation succeeded.
 */
bool transformPose(const std::string& from_frame, const std::string& to_frame,
                   const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose);

/**
 * Transform in_pose from on frame to another
 * @param from_frame initial frame
 * @param to_frame target frame
 * @param in_pose initial pose
 * @param out_pose transformed pose
 * @return true if transformation succeeded.
 */
bool transformPose(const std::string& from_frame, const std::string& to_frame,
                   const geometry_msgs::Pose& in_pose, geometry_msgs::Pose& out_pose);


/**
 * Return the minimum value from a geometry_msgs/Vector3
 * @param v vector
 * @return minimum value
 */
inline double minValue(const geometry_msgs::Vector3& v)
{
  return std::min(std::min(v.x, v.y), v.z);
}

/**
 * Return the maximum value from a geometry_msgs/Vector3
 * @param v vector
 * @return maximum value
 */
inline double maxValue(const geometry_msgs::Vector3& v)
{
  return std::max(std::max(v.x, v.y), v.z);
}

/**
 * Calculate the area of a triangle
 */
float areaTriangle(int x1, int y1, int x2, int y2, int x3, int y3);

/**
 * Check if point P(x, y) lies inside the triangle formed
 * by A(x1, y1), B(x2, y2) and C(x3, y3)
 */
bool insideTriangle(int x1, int y1, int x2, int y2, int x3, int y3, int x, int y);


/**
 * Calculates the slope of a list of points.
 * @param x points x-coordinates
 * @param y points y-coordinates
 * @return the calculated slope
 */
inline double slope(const std::deque<double>& x, const std::deque<double>& y)
{
  const auto n    = x.size();
  const auto s_x  = std::accumulate(x.begin(), x.end(), 0.0);
  const auto s_y  = std::accumulate(y.begin(), y.end(), 0.0);
  const auto s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
  const auto s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
  const auto b    = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
  return b;
}


/**
 * Minimum distance from a point to a line segment
 * @param px point x-coordinate
 * @param py point y-coordinate
 * @param s1x segment's point 1 x-coordinate
 * @param s1y segment's point 1 y-coordinate
 * @param s2x segment's point 2 x-coordinate
 * @param s2y segment's point 2 y-coordinate
 * @return distance
 */
double pointSegmentDistance(double px, double py, double s1x, double s1y, double s2x, double s2y);

/**
 * Do a finite length ray intersects a line segment?
 * @param r1x rays's start point x-coordinate
 * @param r1y rays's start point y-coordinate
 * @param r2x rays's end point x-coordinate
 * @param r2y rays's end point y-coordinate
 * @param s1x segment's point 1 x-coordinate
 * @param s1y segment's point 1 y-coordinate
 * @param s2x segment's point 2 x-coordinate
 * @param s2y segment's point 2 y-coordinate
 * @param ix  intersection point (if any) x-coordinate
 * @param iy  intersection point (if any) y-coordinate
 * @param distance Distance from rays's start to intersection point
 * @return True if ray intersects the segment
 */
bool raySegmentIntersection(double r1x, double r1y, double r2x, double r2y,
                            double s1x, double s1y, double s2x, double s2y,
                            double& ix, double& iy, double& distance);

/**
 * Do a zero-centered, finite length ray intersects a circle?
 * @param rx rays's end point x-coordinate
 * @param ry rays's end point y-coordinate
 * @param cx circle's center x-coordinate
 * @param cy circle's center y-coordinate
 * @param radius circle's radius
 * @param ix closest intersection point (if any) x-coordinate
 * @param iy closest intersection point (if any) y-coordinate
 * @param distance Distance from rays's start (zero) to intersection point
 * @return True if ray intersects the circle
 */
bool rayCircleIntersection(double rx, double ry, double cx, double cy, double radius,
                           double& ix, double& iy, double& distance);

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

} /* namespace mag_common_libs */
