/*
 * Author: Jorge Santos
 */

#include <XmlRpcException.h>
#include <dynamic_reconfigure/config_tools.h>

#include "thorp_toolkit/alternative_config.hpp"

namespace thorp::toolkit
{

AlternativeConfig::AlternativeConfig(const std::string& source_ns, const std::string& target_ns,
                                     const std::string& current_cfg, const std::string& altern_cfg)
  : source_ns_(source_ns), target_ns_(target_ns), current_cfg_(current_cfg), altern_cfg_(altern_cfg)
{
  // Ensure that our namespaces ends with '/', as we will concatenate subsystem names
  if (source_ns_.back() != '/')
    source_ns_.append(1, '/');

  if (target_ns_.back() != '/')
    target_ns_.append(1, '/');

  if (!loadAlternativeConfig(""))
    throw std::runtime_error("Load named configuration from " + source_ns + " namespace failed");
}

bool AlternativeConfig::setConfig(const std::string& configuration)
{
  if (configuration != current_cfg_ && configuration != altern_cfg_)
  {
    ROS_WARN_STREAM("Unknown configuration: '" << configuration << "'; valid names are '" << current_cfg_ << "' and '"
                                               << altern_cfg_ << "'");
    return false;
  }

  bool all_ok = true;
  for (auto& [server_name, subsystem] : reconf_subsystems)
  {
    auto& config = subsystem.config[configuration];
    dynamic_reconfigure::Reconfigure srv;
    srv.request.config = config;
    if (subsystem.srv_client.call(srv))
    {
      ROS_INFO_STREAM("Setting '" << server_name << "' " << configuration << " configuration");
    }
    else
    {
      ROS_ERROR_STREAM("Setting '" << server_name << "' " << configuration
                                   << " configuration failed; dynamic reconfigure call returned error");
      all_ok = false;
    }
  }

  active_cfg_ = configuration;
  return all_ok;
}

bool AlternativeConfig::loadAlternativeConfig(const std::string& ns)
{
  ros::NodeHandle source_nh(ros::names::append(source_ns_, ns));
  ros::NodeHandle target_nh(ros::names::append(target_ns_, ns));

  // read all parameters under the namespace
  XmlRpc::XmlRpcValue configuration;
  if (!source_nh.getParam("", configuration))
  {
    ROS_ERROR_STREAM("Failed to load parameters from " << source_nh.getNamespace());
    return false;
  }

  // load and store them to apply on demand
  auto server = target_nh.getNamespace();
  dynamic_reconfigure::Config altern_cfg;
  dynamic_reconfigure::Config current_cfg;
  std::stringstream ss;

  for (const auto& [param_name, altern_value] : configuration)
  {
    switch (altern_value.getType())
    {
      case XmlRpc::XmlRpcValue::TypeStruct:
      {
        // avoid beginning slash left by append for 0-level namespaces
        const std::string child_ns = ns.empty() ? param_name : ros::names::append(ns, param_name);
        if (!loadAlternativeConfig(child_ns))
        {
          return false;
        }
        break;
      }
      case XmlRpc::XmlRpcValue::TypeBoolean:
        if (bool current_value; target_nh.getParam(param_name, current_value))
        {
          dynamic_reconfigure::ConfigTools::appendParameter<bool>(altern_cfg, param_name, altern_value);
          dynamic_reconfigure::ConfigTools::appendParameter<bool>(current_cfg, param_name, current_value);
        }
        else
        {
          ROS_WARN_STREAM("Parameter '" << target_nh.resolveName(param_name) << "' not found; it will be ignored");
        }
        break;
      case XmlRpc::XmlRpcValue::TypeInt:
        if (int current_value; target_nh.getParam(param_name, current_value))
        {
          dynamic_reconfigure::ConfigTools::appendParameter<int>(altern_cfg, param_name, altern_value);
          dynamic_reconfigure::ConfigTools::appendParameter<int>(current_cfg, param_name, current_value);
        }
        else
        {
          ROS_WARN_STREAM("Parameter '" << target_nh.resolveName(param_name) << "' not found; it will be ignored");
        }
        break;
      case XmlRpc::XmlRpcValue::TypeDouble:
        if (double current_value; target_nh.getParam(param_name, current_value))
        {
          dynamic_reconfigure::ConfigTools::appendParameter<double>(altern_cfg, param_name, altern_value);
          dynamic_reconfigure::ConfigTools::appendParameter<double>(current_cfg, param_name, current_value);
        }
        else
        {
          ROS_WARN_STREAM("Parameter '" << target_nh.resolveName(param_name) << "' not found; it will be ignored");
        }
        break;
      case XmlRpc::XmlRpcValue::TypeString:
        if (std::string current_value; target_nh.getParam(param_name, current_value))
        {
          dynamic_reconfigure::ConfigTools::appendParameter<std::string>(altern_cfg, param_name, altern_value);
          dynamic_reconfigure::ConfigTools::appendParameter<std::string>(current_cfg, param_name, current_value);
        }
        else
        {
          ROS_WARN_STREAM("Parameter '" << target_nh.resolveName(param_name) << "' not found; it will be ignored");
        }
        break;
      default:
        ROS_ERROR_STREAM("Type " << altern_value.getType() << " of parameter " << source_nh.resolveName(param_name)
                                 << " is not supported");
        return false;
    }
    if (altern_value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      ss << "\n  " << param_name << ": " << altern_value;
  }

  // only save non-empty configurations
  if (dynamic_reconfigure::ConfigTools::size(altern_cfg) > 0)
  {
    std::string params_str = ss.str();
    ROS_DEBUG_STREAM("Alternative configuration loaded from " << source_nh.getNamespace() << ":" << params_str);
    ROS_DEBUG_STREAM("Target server is " << server);
    reconf_subsystems[server].config[altern_cfg_] = altern_cfg;
    reconf_subsystems[server].config[current_cfg_] = current_cfg;

    // Create a dynamic reconfigure client per subsystem
    reconf_subsystems[server].srv_client =
        target_nh.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters", true);
    if (!reconf_subsystems[server].srv_client.waitForExistence(ros::Duration(20.0)))
      ROS_WARN_STREAM("'" << server << "' dynamic reconfigure service not available after 20s");
  }

  return true;
}

};  // namespace thorp::toolkit
