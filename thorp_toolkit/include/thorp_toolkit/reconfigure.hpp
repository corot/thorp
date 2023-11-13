/*
 * Author: Jorge Santos
 */

#pragma once

#include <ros/ros.h>

#include <dynamic_reconfigure/Reconfigure.h>

namespace thorp_toolkit
{

/**
 * Dynamic reconfigure of several subsystems sharing a namespace. Reads the current and alternative parameter
 * configurations from a source and target namespaces and a dynamic reconfigure service client to the later,
 * so we can apply one configuration or the other on demand.
 * The alternative configuration must be read first; only the parameters found there are read from the current
 * configuration.
 */
class Reconfigure
{
public:
  /**
   * Constructor: initialize namespaces and configuration names
   * @param source_ns Source namespace, where we read the alternative configuration
   * @param target_ns Target namespace, the configuration we want to replace and restore
   * @param current_cfg Current configuration name, loaded from target namespace
   * @param altern_cfg Alternative configuration name, loaded from source namespace
   */
  Reconfigure(const std::string& ns);

  /**
   * Load an alternative configuration from source namespace and create a dynamic reconfiguration service client
   * on target namespace to the subsystems found.
   * @return True on success, false otherwise
   */
  bool apply();

  /**
   * Load current configuration from target namespace, just the parameters present in the alternative configuration,
   * so we can restore it after setting the alternative configuration.
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
  dynamic_reconfigure::Reconfigure curr_config_;
  dynamic_reconfigure::Reconfigure prev_config_;

  void clear(dynamic_reconfigure::Reconfigure& config);
};

};  // namespace thorp_toolkit
