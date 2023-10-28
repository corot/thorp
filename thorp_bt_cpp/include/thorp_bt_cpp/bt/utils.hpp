#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/tree_node.h>

namespace thorp::bt
{

namespace utils
{
namespace details
{
// SFINAE helper to check if a BT node has ports
template <typename, typename = void>
struct HasPorts : std::false_type
{
};

// SFINAE helper to check if a BT node has ports
template <typename T>
struct HasPorts<T, std::void_t<decltype(T::providedPorts())>> : std::true_type
{
};

// SFINAE helper to check if a type is supported by ros::NodeHandle::getParamCached
template <typename, typename = void>
struct IsParamType : std::false_type
{
};

// SFINAE helper to check if a type is supported by ros::NodeHandle::getParamCached
template <typename T>
struct IsParamType<T, std::void_t<decltype(std::declval<ros::NodeHandle>().getParamCached(
                          std::declval<const std::string&>(), std::declval<T&>()))>> : std::true_type
{
};
}  // namespace details

/**
 * Register the given node with the given factory.
 * Not needed in version 4.
 */
template <typename T, typename... Args>
void registerNode(BT::BehaviorTreeFactory& factory, const ros::NodeHandle& nh, const std::string& name, Args&&... args)
{
  BT::NodeBuilder builder;

  if constexpr (details::HasPorts<T>::value)
  {
    builder = [&](const auto& name, const auto& config)
    { return std::make_unique<T>(name, config, nh, std::forward<Args>(args)...); };
  }
  else
  {
    builder = [&](const auto& name, const auto& /*unused*/)
    { return std::make_unique<T>(name, std::forward<Args>(args)...); };
  }

  factory.registerBuilder<T>(name, builder);
}

/**
 * Gets the value of the given input port.
 * If the value is in the form of ${key}, queries the ROS parameter server.
 * If the value is in the form of {key}, queries the BT blackboard.
 * Otherwise treats the value as a constant.
 */
template <typename T>
void getInput(const BT::TreeNode& node, const ros::NodeHandle& nh, const std::string& key, T& val)
{
  if constexpr (details::IsParamType<T>::value)
  {
    const auto raw = node.getRawPortValue(key);
    const bool is_param = raw.size() >= 4 && raw.rfind("${", 0) == 0 && raw.back() == '}';

    if (is_param)
    {
      const std::string param = raw.substr(2, raw.size() - 3).to_string();
      if (!nh.getParamCached(param, val))
      {
        const std::string msg = "Cannot find parameter " + nh.resolveName(param);
        ROS_ERROR_NAMED(node.name(), msg.c_str());
        throw BT::RuntimeError(msg);
      }

      return;
    }
  }

  if (!node.getInput(key, val))
  {
    const std::string msg = "Cannot find blackboard entry " + key;
    ROS_ERROR_NAMED(node.name(), msg.c_str());
    throw BT::RuntimeError(msg);
  }
}

/**
 * Gets the value of the given input port.
 * If the value is in the form of ${key}, queries the ROS parameter server.
 * If the value is in the form of {key}, queries the BT blackboard.
 * Otherwise treats the value as a constant.
 */
template <typename T>
T getInput(const BT::TreeNode& node, const ros::NodeHandle& nh, const std::string& key)
{
  T val;
  getInput(node, nh, key, val);
  return val;
}

/** Sets the given error in the blackboard. */
void setError(BT::TreeNode& node, unsigned int error)
{
  node.setOutput("error", error);
}
}  // namespace utils
}  // namespace thorp::bt
