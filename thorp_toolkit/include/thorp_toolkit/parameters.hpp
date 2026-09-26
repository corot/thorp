#pragma once

#include <rclcpp/rclcpp.hpp>

#include "thorp_toolkit/common.hpp"

namespace thorp::toolkit
{

/**
 * Get a parameter of the toolkit node, declaring it if needed. The parameter is mandatory: if it has not
 * been set, report it and shut down.
 * @param key parameter name
 * @param o_value parameter value
 * @throw rclcpp::exceptions::ParameterUninitializedException or InvalidParameterTypeException
 */
template <typename T>
void getParam(const std::string& key, T& o_value)
{
  try
  {
    if (!node()->has_parameter(key))
    {
      node()->declare_parameter<T>(key);
    }
    o_value = node()->get_parameter(key).get_value<T>();
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(logger(), "Cannot get parameter %s from node %s: %s", key.c_str(), node()->get_name(), e.what());
    rclcpp::shutdown();
    throw;
  }
}

inline void getParam(const std::string& key, rclcpp::Duration& o_value)
{
  double val;
  getParam(key, val);
  o_value = rclcpp::Duration::from_seconds(val);
}

/**
 * Get a parameter of the toolkit node, declaring it if needed, or the default value if it has not been set
 * @param key parameter name
 * @param o_value parameter value
 * @param default_value value to use if the parameter has not been set
 */
template <typename T>
void getParam(const std::string& key, T& o_value, const T& default_value)
{
  if (!node()->has_parameter(key))
  {
    node()->declare_parameter<T>(key, default_value);
  }
  o_value = node()->get_parameter(key).get_value<T>();
}

template <typename T>
T getParam(const std::string& key)
{
  T o_value;
  getParam(key, o_value);
  return o_value;
}

} /* namespace thorp::toolkit */
