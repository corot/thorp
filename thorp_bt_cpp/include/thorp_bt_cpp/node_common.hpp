#pragma once

#include <string>
#include <tuple>

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/exceptions.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include "thorp_bt_cpp/bt_runner.hpp"
#include "thorp_bt_cpp/type_converters.hpp"

namespace thorp::bt
{
/**
 * @brief Struct that will, upon creation, register a builder for the owner custom node to the BT factory.
 * You need to add this struct as a static attribute to each of your custom BT nodes, so they get registered
 * on static initialization.
 * The instantiation of the builder will happen later, when loading each tree from files.
 * Extra arguments are passed to the node constructor after its name, e.g. the default action name for Nav2's
 * BtActionNode; the builder keeps copies of them, as it runs long after registration.
 */
template <typename T, typename... Args>
struct NodeRegister
{
  explicit NodeRegister(const std::string& name, Args... args)
  {
    BT::NodeBuilder builder = [args = std::make_tuple(args...)](const std::string& name,
                                                                const BT::NodeConfig& config) {
      return std::apply([&](const auto&... node_args) { return std::make_unique<T>(name, node_args..., config); },
                        args);
    };
    Runner::bt_factory_.registerBuilder<T>(name, builder);
  }
};

/**
 * Macro to register a ClassName node. Node ID will be the same as the class name.
 * To be called inside the node class.
 */
#define BT_REGISTER_NODE(ClassName) inline static NodeRegister<ClassName> reg_{ #ClassName };

/**
 * Macro to register a ClassName node derived from Nav2's BtActionNode, with its default action name.
 * To be called inside the node class.
 */
#define BT_REGISTER_ACTION_NODE(ClassName, action_name)                                                               \
  inline static NodeRegister<ClassName, std::string> reg_{ #ClassName, action_name };

/**
 * Macro to register a template version of ClassName node (you need to specify a unique ID for each version).
 * To be called outside the node class but within `thorp::bt::` namespace.
 * Requires attribute `static NodeRegister<ClassName<T>> reg_;` to the template class
 */
#define BT_REGISTER_TEMPLATE_NODE(ClassName, ID)                                                                       \
  template <>                                                                                                          \
  NodeRegister<ClassName> ClassName::reg_{ ID };

/**
 * The ROS node running the tree, that the runner and the server put on the blackboard as "node".
 */
inline rclcpp::Node::SharedPtr rosNode(const BT::TreeNode& node)
{
  return node.config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

/**
 * A logger named after the tree node, child of the ROS node's logger.
 */
inline rclcpp::Logger logger(const BT::TreeNode& node)
{
  return rosNode(node)->get_logger().get_child(node.name());
}

/**
 * A port's value, or a BT::RuntimeError saying which port and key had none.
 *
 * Use it instead of *getInput<T>(...): Expected's operator* only asserts, so in a Release build
 * an unset port is undefined behavior, where this is an exception bt_server turns into an
 * aborted goal that names the missing key.
 */
template <typename T>
T requireInput(const BT::TreeNode& node, const std::string& port)
{
  auto value = node.getInput<T>(port);
  if (!value)
  {
    throw BT::RuntimeError(node.name(), ": ", value.error());
  }
  return std::move(value.value());
}

}  // namespace thorp::bt
