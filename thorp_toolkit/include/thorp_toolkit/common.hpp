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
#include <geometry_msgs/Vector3.h>


namespace thorp_toolkit
{


bool transformPose(const std::string& in_frame, const std::string& out_frame,
                   const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose);

bool transformPose(const std::string& in_frame, const std::string& out_frame,
                   const geometry_msgs::Pose& in_pose, geometry_msgs::Pose& out_pose);


// Template functions

template <typename T>
T median(const std::vector<T>& v) {
  // Return the median element of a vector
  std::vector<T> c = v;  // clone, as nth_element will short half of the vector
  std::nth_element(c.begin(), c.begin() + c.size()/2, c.end());
  return c[c.size()/2];
};

// non-template version for median; is faster, but I think mostly due to not cloning the vector
//double median(std::vector<double>& values)
//{
////  std::vector<double> AB;
////  for (int i = 0; i<10000; i++)
////    AB.push_back(rand());
////  std::vector<double> CD = AB;
////  ros::Time t1 = ros::Time::now();
////  double r1 = mtk::median(AB);
////  ros::Time t2 = ros::Time::now();
////  ros::Time t3 = ros::Time::now();
////  double r2 = median(CD);
////  ros::Time t4 = ros::Time::now();
////  ROS_WARN("%f  %f     %f  %f", r1, r2, (t2-t1).toSec(), (t4-t3).toSec());
////  A bit faster because no clone... can I do const/no const versions of the mtk impl???
//  std::vector<double>::iterator first = values.begin();
//  std::vector<double>::iterator last = values.end();
//  std::vector<double>::iterator middle = first + (last - first) / 2;
//  std::nth_element(first, middle, last); // short values till middle one
//  return *middle;
//}

double min_dim(const geometry_msgs::Vector3& v);

double max_dim(const geometry_msgs::Vector3& v);

float area(int x1, int y1, int x2, int y2, int x3, int y3);

/* A function to check whether point P(x, y) lies inside the triangle formed
   by A(x1, y1), B(x2, y2) and C(x3, y3) */
bool isInside(int x1, int y1, int x2, int y2, int x3, int y3, int x, int y);

/*
 * Determine barycentric coordinates
 */
int orient2d(int Ax, int Ay, int Bx, int By, int Cx, int Cy);


} /* namespace thorp_toolkit */
