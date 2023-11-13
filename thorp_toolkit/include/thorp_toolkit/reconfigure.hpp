/*
 * Author: Jorge Santos
 */

#pragma once

#include <ros/ros.h>

#include <dynamic_reconfigure/Reconfigure.h>

namespace thorp_toolkit
{

/**
 * Dynamic reconfigure for a single namespace.
 * The advantage over dynamic_reconfigure::Client is that this can be used without template.
 */
class Reconfigure
{
public:
  /**
   * Constructor: create a service client to change the configuration on the given namespace
   * @param ns Target namespace we plan to reconfigure
   */
  Reconfigure(const std::string& ns);

  /**
   * Apply the configuration added with addParam methods.
   * @return True on success, false otherwise
   */
  bool apply();

  /**
   * Revert the configuration to the original values.
   * @return True on success, false otherwise
   */
  bool revert();

  bool addParam(const std::string& name, bool value);

  bool addParam(const std::string& name, int value);

  bool addParam(const std::string& name, double value);

  bool addParam(const std::string& name, const std::string& value);

private:
  ros::NodeHandle nh_;
  ros::ServiceClient srv_client_;
  dynamic_reconfigure::Config curr_config_;
  dynamic_reconfigure::Config prev_config_;

  void clear(dynamic_reconfigure::Config& config);
};

};  // namespace thorp_toolkit
