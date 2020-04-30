/*
 * Author: Jorge Santos
 */

#pragma once

#include <deque>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <thorp_toolkit/math.hpp>
namespace ttk = thorp_toolkit;

namespace thorp_manipulation
{

class JointStateWatchdog
{
public:
  JointStateWatchdog();
  ~JointStateWatchdog();

  void jointStateCB(const sensor_msgs::JointState& msg);

  double gripperOpening() { return ttk::mean(gripper_opening); }
  double gripperEffort() { return ttk::mean(gripper_effort); }

private:
  double gripper_pad_width;
  double gripper_finger_length;
  double gripper_min_opening;
  double gripper_max_opening;
  double gripper_center;
  bool gripper_invert;
  std::string gripper_joint;
  int gripper_joint_index = -1;

  const unsigned int HISTORY_SIZE = 10;
  std::deque<double> gripper_opening;
  std::deque<double> gripper_effort;

  ros::Subscriber joint_state_sub;
  ros::Publisher joint_state_pub;
};

};
