/*
 * Author: Jorge Santos
 */

#include "thorp_toolkit/reconfigure.hpp"

namespace thorp_toolkit
{

Reconfigure::Reconfigure(const std::string& ns) : nh_(ns)
{
  // Create a dynamic reconfigure client per subsystem
  srv_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters", true);
  if (!srv_client_.waitForExistence(ros::Duration(10.0)))
    throw ros::Exception("'" + ns + "' dynamic reconfigure service not available after 10s");
}

bool Reconfigure::apply()
{
  bool succeeded = srv_client_.call(curr_config_);
  if (succeeded)
  {
    ROS_INFO_STREAM("Updated '" << nh_.getNamespace() << "' configuration");
  }
  else
  {
    ROS_ERROR_STREAM("Updating '" << nh_.getNamespace() << " configuration failed");
  }
  clear(curr_config_);
  return succeeded;
}

bool Reconfigure::revert()
{
  bool succeeded = srv_client_.call(prev_config_);
  if (succeeded)
  {
    ROS_INFO_STREAM("Reverted '" << nh_.getNamespace() << "' configuration");
  }
  else
  {
    ROS_ERROR_STREAM("Reverting '" << nh_.getNamespace() << " configuration failed");
  }
  clear(curr_config_);
  return succeeded;
}

bool Reconfigure::addParam(const std::string& name, bool value)
{
  dynamic_reconfigure::BoolParameter param;
  param.name = name;
  if (!nh_.getParam(nh_.getNamespace() + "/" + param.name, (bool&)param.value))
  {
    ROS_ERROR_STREAM("Cannot read current value for parameter '" << name << "'; ignoring");
    return false;
  }
  prev_config_.request.config.bools.push_back(param);
  param.value = value;
  curr_config_.request.config.bools.push_back(param);
  return true;
}

bool Reconfigure::addParam(const std::string& name, int value)
{
  dynamic_reconfigure::IntParameter param;
  param.name = name;
  if (!nh_.getParam(nh_.getNamespace() + "/" + param.name, param.value))
  {
    ROS_ERROR_STREAM("Cannot read current value for parameter '" << name << "'; ignoring");
    return false;
  }
  prev_config_.request.config.ints.push_back(param);
  param.value = value;
  curr_config_.request.config.ints.push_back(param);
  return true;
}

bool Reconfigure::addParam(const std::string& name, double value)
{
  dynamic_reconfigure::DoubleParameter param;
  param.name = name;
  if (!nh_.getParam(nh_.getNamespace() + "/" + param.name, param.value))
  {
    ROS_ERROR_STREAM("Cannot read current value for parameter '" << name << "'; ignoring");
    return false;
  }
  prev_config_.request.config.doubles.push_back(param);
  param.value = value;
  curr_config_.request.config.doubles.push_back(param);
  return true;
}

bool Reconfigure::addParam(const std::string& name, const std::string& value)
{
  dynamic_reconfigure::StrParameter param;
  param.name = name;
  if (!nh_.getParam(nh_.getNamespace() + "/" + param.name, param.value))
  {
    ROS_ERROR_STREAM("Cannot read current value for parameter '" << name << "'; ignoring");
    return false;
  }
  prev_config_.request.config.strs.push_back(param);
  param.value = value;
  curr_config_.request.config.strs.push_back(param);
  return true;
}

void Reconfigure::clear(dynamic_reconfigure::Reconfigure& config)
{
  config.request.config.bools.clear();
  config.request.config.ints.clear();
  config.request.config.doubles.clear();
  config.request.config.strs.clear();
}

};  // namespace thorp_toolkit
