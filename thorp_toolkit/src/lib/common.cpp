/*
 * common.cpp
 *
 *  Created on: Feb 20, 2016
 *      Author: jorge
 */

#include "thorp_toolkit/common.hpp"


namespace thorp_toolkit
{


bool transformPose(const std::string& in_frame, const std::string& out_frame,
                   const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose)
{
  try
  {
    static tf::TransformListener tf_listener;
    tf_listener.waitForTransform(in_frame, out_frame, ros::Time(0.0), ros::Duration(1.0));
    tf_listener.transformPose(out_frame, in_pose, out_pose);
    return true;
  }
  catch (tf::InvalidArgument& e)
  {
    ROS_ERROR("[thorp toolkit] Pose to transform has invalid orientation: %s", e.what());
    return false;
  }
  catch (tf::TransformException& e)
  {
    ROS_ERROR("[thorp toolkit] Could not get '%s' to '%s' transform: %s",
              in_frame.c_str(), out_frame.c_str(), e.what());
    return false;
  }
}

bool transformPose(const std::string& in_frame, const std::string& out_frame,
                   const geometry_msgs::Pose& in_pose, geometry_msgs::Pose& out_pose)
{
  geometry_msgs::PoseStamped in_stamped;
  geometry_msgs::PoseStamped out_stamped;

  in_stamped.header.frame_id = in_frame;
  in_stamped.pose = in_pose;
  if (transformPose(in_frame, out_frame, in_stamped, out_stamped))
  {
    out_pose = out_stamped.pose;
    return true;
  }

  return false;
}

double min_dim(const geometry_msgs::Vector3& v)
{
  return std::min(std::min(v.x, v.y), v.z);
}

double max_dim(const geometry_msgs::Vector3& v)
{
  return std::max(std::max(v.x, v.y), v.z);
}


/* TODO REVIEW THESE FUNS */
float area(int x1, int y1, int x2, int y2, int x3, int y3)
{
   return fabs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0);
}

bool isInside(int x1, int y1, int x2, int y2, int x3, int y3, int x, int y)
{
   /* Calculate area of triangle ABC */
   float A = area (x1, y1, x2, y2, x3, y3);

   /* Calculate area of triangle PBC */
   float A1 = area (x, y, x2, y2, x3, y3);

   /* Calculate area of triangle PAC */
   float A2 = area (x1, y1, x, y, x3, y3);

   /* Calculate area of triangle PAB */
   float A3 = area (x1, y1, x2, y2, x, y);

   /* Check if sum of A1, A2 and A3 is same as A */
   return (A == A1 + A2 + A3);
}

int orient2d(int Ax, int Ay, int Bx, int By, int Cx, int Cy)
{
    return (Bx-Ax)*(Cy-Ay) - (By-Ay)*(Cx-Ax);
}


} /* namespace thorp_toolkit */
