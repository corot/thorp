/*
 * common.hpp
 *
 *  Created on: Feb 20, 2016
 *      Author: jorge
 */

#pragma once


#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


namespace thorp_toolkit
{


bool transformPose(const std::string& in_frame, const std::string& out_frame,
                   const geometry_msgs::Pose& in_pose, geometry_msgs::Pose& out_pose)
{
  geometry_msgs::PoseStamped in_stamped;
  geometry_msgs::PoseStamped out_stamped;

  in_stamped.header.frame_id = in_frame;
  in_stamped.pose = in_pose;
  try
  {
    static tf::TransformListener tf_listener_;
    tf_listener_.waitForTransform(in_frame, out_frame, ros::Time(0.0), ros::Duration(1.0));
    tf_listener_.transformPose(out_frame, in_stamped, out_stamped);
    out_pose = out_stamped.pose;

//      // Some verifications...
//      // tables sometimes have orientations with all-nan values, but assertQuaternionValid lets them go!   TODO recheck issue
//      tf::assertQuaternionValid(out_stamped.pose.orientation);
//      if (std::isnan(out_pose.orientation.x) ||
//          std::isnan(out_pose.orientation.y) ||
//          std::isnan(out_pose.orientation.z) ||
//          std::isnan(out_pose.orientation.w))
//        throw tf::InvalidArgument("Quaternion contains nan values");

    return true;
  }
  catch (tf::InvalidArgument& e)
  {
    ROS_ERROR("[object detection] Transformed pose has invalid orientation: %s", e.what());
    return false;
  }
  catch (tf::TransformException& e)
  {
    ROS_ERROR("[object detection] Could not get sensor to arm transform: %s", e.what());
    return false;
  }
}


// Template functions


template <typename T> T median(const std::vector<T>& v) {
  // Return the median element of a vector
  std::vector<T> c = v;  // clone, as nth_element will short half of the vector
  std::nth_element(c.begin(), c.begin() + c.size()/2, c.end());
  return c[c.size()/2];
};

double median(std::vector<double>& values)
{
//  std::vector<double> AB;
//  for (int i = 0; i<10000; i++)
//    AB.push_back(rand());
//  std::vector<double> CD = AB;
//  ros::Time t1 = ros::Time::now();
//  double r1 = mtk::median(AB);
//  ros::Time t2 = ros::Time::now();
//  ros::Time t3 = ros::Time::now();
//  double r2 = median(CD);
//  ros::Time t4 = ros::Time::now();
//  ROS_WARN("%f  %f     %f  %f", r1, r2, (t2-t1).toSec(), (t4-t3).toSec());
//  A bit faster because no clone... can I do const/no const versions of the mtk impl???
  std::vector<double>::iterator first = values.begin();
  std::vector<double>::iterator last = values.end();
  std::vector<double>::iterator middle = first + (last - first) / 2;
  std::nth_element(first, middle, last); // short values till middle one
  return *middle;
}


} /* namespace thorp_toolkit */
