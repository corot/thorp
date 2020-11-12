/*
 * geometry.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#include <tf/transform_listener.h>

#include "mag_common_cpp_libs/geometry.hpp"


namespace mag_common_libs
{


bool transformPose(const std::string& from_frame, const std::string& to_frame,
                   const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose)
{
  try
  {
    static tf::TransformListener tf_listener;
    tf_listener.waitForTransform(to_frame, from_frame, ros::Time(0.0), ros::Duration(1.0));
    tf_listener.transformPose(to_frame, in_pose, out_pose);
    return true;
  }
  catch (tf::InvalidArgument& e)
  {
    ROS_ERROR("Pose to transform has invalid orientation: %s", e.what());
    return false;
  }
  catch (tf::TransformException& e)
  {
    ROS_ERROR("Could not get '%s' to '%s' transform: %s", from_frame.c_str(), to_frame.c_str(), e.what());
    return false;
  }
}

bool transformPose(const std::string& from_frame, const std::string& to_frame,
                   const geometry_msgs::Pose& in_pose, geometry_msgs::Pose& out_pose)
{
  geometry_msgs::PoseStamped in_stamped;
  geometry_msgs::PoseStamped out_stamped;

  in_stamped.header.frame_id = from_frame;
  in_stamped.pose = in_pose;
  if (transformPose(from_frame, to_frame, in_stamped, out_stamped))
  {
    out_pose = out_stamped.pose;
    return true;
  }

  return false;
}


float areaTriangle(int x1, int y1, int x2, int y2, int x3, int y3)
{
  return fabs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0);
}

bool insideTriangle(int x1, int y1, int x2, int y2, int x3, int y3, int x, int y)
{
  /* Calculate area of triangle ABC */
  float A = areaTriangle(x1, y1, x2, y2, x3, y3);

  /* Calculate area of triangle PBC */
  float A1 = areaTriangle(x, y, x2, y2, x3, y3);

  /* Calculate area of triangle PAC */
  float A2 = areaTriangle(x1, y1, x, y, x3, y3);

  /* Calculate area of triangle PAB */
  float A3 = areaTriangle(x1, y1, x2, y2, x, y);

  /* Check if sum of A1, A2 and A3 is same as A */
  return (A == A1 + A2 + A3);
}

double pointSegmentDistance(double px, double py, double s1x, double s1y, double s2x, double s2y)
{
  // Return minimum distance between line segment s1-s2 and point p

  double l = distance2D(s1x, s1y, s2x, s2y);    // i.e. |p2 - p1|^2
  if (l == 0.0)
    return distance2D(px, py, s1x, s1y);        // s1 == s2 case

  // Consider the line extending the segment, parameterized as s1 + t (s2 - s1).
  // We find projection of point p onto the line.
  // It falls where t = [(p - s1) . (s2 - s1)] / |s2 - s1|^2
  double t = (- s1x * (s2x - s1x) - s1y * (s2y - s1y)) / l;
  if (t < 0.0)                           // Beyond the s1 end of the segment
    return distance2D(s1x, s1y);

  if (t > 1.0)                           // Beyond the s2 end of the segment
    return distance2D(s2x, s2y);

  // Projection falls on the segment
  return distance2D(s1x + t * (s2x - s1x), s1y + t * (s2y - s1y));
}

bool raySegmentIntersection(double r1x, double r1y, double r2x, double r2y,
                            double s1x, double s1y, double s2x, double s2y,
                            double& ix, double& iy, double& distance)
{
  // Make sure the lines aren't parallel
  if ((r2y - r1y) / (r2x - r1x) != (s2y - s1y) / (s2x - s1x))
  {
    double d = (((r2x - r1x) * (s2y - s1y)) - (r2y - r1y) * (s2x - s1x));
    if (d != 0)
    {
      double r = (((r1y - s1y) * (s2x - s1x)) - (r1x - s1x) * (s2y - s1y)) / d;
      double s = (((r1y - s1y) * (r2x - r1x)) - (r1x - s1x) * (r2y - r1y)) / d;
      if (r >= 0)
      {
        if (s >= 0 && s <= 1)
        {
          ix = r1x + r * (r2x - r1x);
          iy = r1y + r * (r2y - r1y);
          distance = distance2D(ix, iy);
          return true;
        }
      }
    }
  }
  return false;
}

bool rayCircleIntersection(double rx, double ry, double cx, double cy, double radius,
                           double& ix, double& iy, double& distance)
{
  double a = rx * rx + ry * ry;
  double bBy2 = rx * cx + ry * cy;
  double c = cx * cx + cy * cy - radius * radius;

  double pBy2 = bBy2 / a;
  double q = c / a;

  double discriminant = pBy2 * pBy2 - q;
  if (discriminant < 0)
    return false;

  // if disc == 0 ... dealt with later
  double tmpSqrt = std::sqrt(discriminant);
  double abScalingFactor1 = -pBy2 + tmpSqrt;
  double abScalingFactor2 = -pBy2 - tmpSqrt;

  ix = - rx * abScalingFactor1;
  iy = - ry * abScalingFactor1;
  distance = distance2D(ix, iy);

  // discard the backward-pointing half of the ray
  if ((ix*rx < 0.0) && (iy*ry < 0.0))
    return false;

  if (discriminant == 0)  // abScalingFactor1 == abScalingFactor2
    return true;

  // Check if the second intersection point is close (naively inefficient)
  double i2x = - rx * abScalingFactor2;
  double i2y = - ry * abScalingFactor2;
  double distance2 = distance2D(i2x, i2y);

  if (distance2 < distance)
  {
    ix = i2x;
    iy = i2y;
    distance = distance2;
  }

  return true;
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


} /* namespace mag_common_libs */
