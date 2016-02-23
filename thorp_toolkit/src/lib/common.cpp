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


} /* namespace thorp_toolkit */
