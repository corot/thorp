#pragma once

#include <ros/ros.h>

namespace thorp::toolkit
{

template <typename T>
void getParam(const std::string& key, T& o_value, ros::NodeHandle nh = ros::NodeHandle("~"))
{
  if (!nh.getParam(key, o_value))
  {
    ROS_ERROR_STREAM("Cannot find parameter " << key << " in namespace " << nh.getNamespace() << "!");
    ros::shutdown();
    assert(false);
  }
}

inline void getParam(const std::string& key, ros::Duration& o_value, ros::NodeHandle nh = ros::NodeHandle("~"))
{
  float val;
  getParam(key, val, nh);
  o_value = ros::Duration(val);
}

template <typename T>
void getParam(const std::string& key, T& o_value, const T& default_value, ros::NodeHandle nh = ros::NodeHandle("~"))
{
  if (!nh.getParam(key, o_value))
  {
    ROS_INFO_STREAM("Cannot find parameter " << key << " in namespace " << nh.getNamespace()
                                             << ", assigning default value: " << default_value);
    o_value = default_value;
  }
}

template <typename T>
T getParam(const std::string& key, ros::NodeHandle nh = ros::NodeHandle("~"))
{
  T o_value;
  getParam(key, o_value, nh);
  return o_value;
}

} /* namespace thorp::toolkit */
