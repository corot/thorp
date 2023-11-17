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
class AlternativeConfig
{
public:
  /**
   * Constructor: initialize namespaces and configuration names
   * @param source_ns Source namespace, where we read the alternative configuration
   * @param target_ns Target namespace, the configuration we want to replace and restore
   * @param current_cfg Current configuration name, loaded from target namespace
   * @param altern_cfg Alternative configuration name, loaded from source namespace
   */
  AlternativeConfig(const std::string& source_ns, const std::string& target_ns,
                    const std::string& current_config, const std::string& altern_config);

  /**
   * Load one of the previously load configurations, previously defined with altern_config and current_config
   * constructor arguments
   * @param configuration Name of the configuration to set
   * @return True on success, false otherwise
   */
  bool setConfig(const std::string& configuration);

  /**
   * Get the name of the currently loaded configuration
   * @return Name of the currently loaded configuration
   */
  std::string getActiveConfig()
  {
    return active_cfg_;
  }

private:
  std::string source_ns_;
  std::string target_ns_;

  std::string current_cfg_;
  std::string altern_cfg_;
  std::string active_cfg_;

  typedef struct
  {
    ros::ServiceClient srv_client;
    std::map<std::string, dynamic_reconfigure::Config> config;
  } ReconfigurableSubsystem;

  std::map<std::string, ReconfigurableSubsystem> reconf_subsystems;

  /**
   * Load an alternative configuration from source namespace and create a dynamic reconfiguration service client
   * on target namespace to the subsystems found.
   * For each parameter, we also get the current value, so we can restore the original configuration.
   * @return True on success, false otherwise
   */
  bool loadAlternativeConfig(const std::string& ns);
};

};  // namespace thorp_toolkit
