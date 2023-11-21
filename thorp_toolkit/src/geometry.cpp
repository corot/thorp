/*
 * Author: Jorge Santos
 */

#include "thorp_toolkit/geometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

namespace thorp_toolkit
{

char ___buffers___[10][256];
int ___next_buffer___ = -1;

double dot(const std::array<geometry_msgs::Point, 2>& v1, const std::array<geometry_msgs::Point, 2>& v2)
{
  Eigen::Vector3d a, b, c, d;
  tf2::fromMsg(v1[0], a);
  tf2::fromMsg(v1[1], b);
  tf2::fromMsg(v2[0], c);
  tf2::fromMsg(v2[1], d);

  return (b - a).dot(d - c);
}

double distance2D(const std::array<geometry_msgs::Point, 2>& line_points, const geometry_msgs::Point& point)
{
  Eigen::Vector3d p, p0, p1;
  tf2::fromMsg(point, p);
  tf2::fromMsg(line_points[0], p0);
  tf2::fromMsg(line_points[1], p1);

  // 2D
  p.z() = 0;
  p0.z() = 0;
  p1.z() = 0;

  Eigen::Vector3d p0p1 = p1 - p0;
  Eigen::Vector3d p0p = p - p0;
  double seg_len = p0p1.norm();
  double t = p0p.dot(p0p1) / seg_len;
  t = std::max(0.0, std::min(1.0, t));
  return (p0p - (p0p1 * t)).norm();
}

geometry_msgs::Pose createPose(double x, double y, double yaw)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return pose;
}

geometry_msgs::PoseStamped createPoseStamped(double x, double y, double yaw, const std::string& frame)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose = createPose(x, y, yaw);
  return pose;
}

geometry_msgs::Pose2D toPose2D(const geometry_msgs::Pose& pose)
{
  geometry_msgs::Pose2D pose_2d;
  pose_2d.x = pose.position.x;
  pose_2d.y = pose.position.y;
  pose_2d.theta = yaw(pose);
  return pose_2d;
}

geometry_msgs::Pose toPose3D(const geometry_msgs::Pose2D& pose)
{
  return createPose(pose.x, pose.y, pose.theta);
}

geometry_msgs::Point32 toPoint(const geometry_msgs::Point& point)
{
  geometry_msgs::Point32 point32;
  point32.x = point.x;
  point32.y = point.y;
  return point32;
}

geometry_msgs::Point toPoint(const geometry_msgs::Point32& point32)
{
  geometry_msgs::Point point;
  point.x = point32.x;
  point.y = point32.y;
  return point;
}


std::string toStr3D(const geometry_msgs::Vector3& vector)
{
  return toCStr3D(vector);
}

std::string toStr3D(const geometry_msgs::Vector3Stamped& vector)
{
  return toCStr3D(vector.vector);
}

std::string toStr2D(const geometry_msgs::Point& point)
{
  return toCStr2D(point);
}

std::string toStr2D(const geometry_msgs::PointStamped& point)
{
  return toCStr2D(point.point);
}

std::string toStr3D(const geometry_msgs::Point& point)
{
  return toCStr3D(point);
}

std::string toStr3D(const geometry_msgs::PointStamped& point)
{
  return toCStr3D(point.point);
}

std::string toStr2D(const geometry_msgs::Pose& pose)
{
  return toCStr2D(pose);
}

std::string toStr2D(const geometry_msgs::PoseStamped& pose)
{
  return toCStr2D(pose.pose);
}

std::string toStr2D(const tf::Stamped<tf::Pose>& pose)
{
  return toCStr2D(pose);
}

std::string toStr3D(const geometry_msgs::Pose& pose)
{
  return toCStr3D(pose);
}

std::string toStr3D(const geometry_msgs::PoseStamped& pose)
{
  return toCStr3D(pose.pose);
}

std::string toStr3D(const tf::Stamped<tf::Pose>& pose)
{
  return toCStr3D(pose);
}


const char* toCStr3D(const geometry_msgs::Vector3& vector)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f", vector.x, vector.y, vector.z);
  return ___buffers___[___next_buffer___];
}

const char* toCStr3D(const geometry_msgs::Vector3Stamped& vector)
{
  return toCStr3D(vector.vector);
}

const char* toCStr2D(const geometry_msgs::Point& point)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f", point.x, point.y);
  return ___buffers___[___next_buffer___];
}

const char* toCStr2D(const geometry_msgs::PointStamped& point)
{
  return toCStr2D(point.point);
}

const char* toCStr3D(const geometry_msgs::Point& point)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f", point.x, point.y, point.z);
  return ___buffers___[___next_buffer___];
}

const char* toCStr3D(const geometry_msgs::PointStamped& point)
{
  return toCStr3D(point.point);
}

const char* toCStr2D(const geometry_msgs::Pose& pose)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f", pose.position.x, pose.position.y, yaw(pose));
  return ___buffers___[___next_buffer___];
}

const char* toCStr2D(const geometry_msgs::PoseStamped& pose)
{
  return toCStr2D(pose.pose);
}

const char* toCStr2D(const tf::Stamped<tf::Pose>& pose)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f", pose.getOrigin().x(), pose.getOrigin().y(), yaw(pose));
  return ___buffers___[___next_buffer___];
}

const char* toCStr3D(const geometry_msgs::Pose& pose)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f,  %.2f, %.2f, %.2f",
          pose.position.x, pose.position.y, pose.position.z, roll(pose), pitch(pose), yaw(pose));
  return ___buffers___[___next_buffer___];
}

const char* toCStr3D(const geometry_msgs::PoseStamped& pose)
{
  return toCStr3D(pose.pose);
}

const char* toCStr3D(const tf::Stamped<tf::Pose>& pose)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f,  %.2f, %.2f, %.2f",
          pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(), roll(pose), pitch(pose), yaw(pose));
  return ___buffers___[___next_buffer___];
}

bool clipSegment(double edge_left, double edge_right, double edge_bottom, double edge_top,
                 double x0src, double y0src, double x1src, double y1src,
                 double& x0clip, double& y0clip, double& x1clip, double& y1clip)
{
  double t0 = 0.0;
  double t1 = 1.0;
  double xdelta = x1src - x0src;
  double ydelta = y1src - y0src;
  double p, q, r;

  for (int edge = 0; edge < 4; edge++)
  {   // Traverse through left, right, bottom, top edges.
    if (edge == 0)
    {
      p = -xdelta;
      q = -(edge_left - x0src);
    }
    if (edge == 1)
    {
      p = xdelta;
      q = (edge_right - x0src);
    }
    if (edge == 2)
    {
      p = -ydelta;
      q = -(edge_bottom - y0src);
    }
    if (edge == 3)
    {
      p = ydelta;
      q = (edge_top - y0src);
    }
    r = q / p;
    if (p == 0 && q < 0) return false;   // Don't draw line at all. (parallel line outside)

    if (p < 0)
    {
      if (r > t1) return false;          // Don't draw line at all.
      else if (r > t0) t0 = r;           // Line is clipped!
    }
    else if (p > 0)
    {
      if (r < t0) return false;          // Don't draw line at all.
      else if (r < t1) t1 = r;           // Line is clipped!
    }
  }

  x0clip = x0src + t0 * xdelta;
  y0clip = y0src + t0 * ydelta;
  x1clip = x0src + t1 * xdelta;
  y1clip = y0src + t1 * ydelta;

  return true;        // (clipped) line is drawn
}


double enclosedArea(const geometry_msgs::PoseStamped& pose_a,
                    const geometry_msgs::PoseStamped& pose_b,
                    const geometry_msgs::PoseStamped& pose_c)
{
  return std::abs((pose_a.pose.position.x * (pose_b.pose.position.y - pose_c.pose.position.y) +
                   pose_b.pose.position.x * (pose_c.pose.position.y - pose_a.pose.position.y) +
                   pose_c.pose.position.x * (pose_a.pose.position.y - pose_b.pose.position.y)) / 2);
}

double curvature(const geometry_msgs::PoseStamped& pose_a,
                 const geometry_msgs::PoseStamped& pose_b,
                 const geometry_msgs::PoseStamped& pose_c)
{
  double area = enclosedArea(pose_a, pose_b, pose_c);
  double dist_ab = distance2D(pose_a, pose_b);
  double dist_bc = distance2D(pose_b, pose_c);
  double dist_ca = distance2D(pose_c, pose_a);
  return 4 * area / (dist_ab * dist_bc * dist_ca);
}

} /* namespace thorp_toolkit */
