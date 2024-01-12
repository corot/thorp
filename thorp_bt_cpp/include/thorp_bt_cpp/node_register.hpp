#pragma once

#include <type_traits>

#include <ros/ros.h>
#include <behaviortree_cpp_v3/tree_node.h>

#include "thorp_bt_cpp/bt_runner.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

namespace thorp::bt
{
/**
 * @brief Struct that will, upon creation, register a builder for the owner custom node to the BT factory.
 * You need to add this struct as a static attribute to each of your custom BT nodes, so they get registered
 * on static initialization.
 * The instantiation of the builder will happen later, when loading each tree from files.
 */
template <typename T, typename... Args>
struct NodeRegister
{
  NodeRegister(const std::string& name, Args&&... args)
  {
    ROS_DEBUG_STREAM_NAMED("NodeRegister", name << " node registered");

    BT::NodeBuilder builder = [&](const auto& name, const auto& config)
    { return std::make_unique<T>(name, config, std::forward<Args>(args)...); };

    Runner::bt_factory_.registerBuilder<T>(name, builder);
  }
};

/**
 * Macro to register a ClassName node. Node ID will be the same as the class name.
 * To be called inside the node class.
 */
#define BT_REGISTER_NODE(ClassName) inline static NodeRegister<ClassName> reg_{ #ClassName };

/**
 * Macro to register a template version of ClassName node (you need to specify a unique ID for each version).
 * To be called outside the node class but within `thorp::bt::` namespace.
 * Requires attribute `static NodeRegister<ClassName<T>> reg_;` to the template class
 */
#define BT_REGISTER_TEMPLATE_NODE(ClassName, ID)                                                                       \
  template <>                                                                                                          \
  NodeRegister<ClassName> ClassName::reg_{ ID };

}  // namespace thorp::bt
