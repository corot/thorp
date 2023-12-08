#pragma once

#include "thorp_bt_cpp/bt/actions/action_client_node.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

extern BT::BehaviorTreeFactory bt_factory_;

/**
 * @brief Register a BT node to the factory
 */
template <typename T>
struct NodeRegister
{
  NodeRegister(std::string const& name)
  {
    ROS_WARN_STREAM(name << " node registered");
    std::cout << name << " node registered" << "\n";
    bt_factory_.registerNodeType<T>(name);
  }
};


/**
 * @brief Register a BT node to the factory
 */
template <typename T>
struct ROSActionClientNodeRegister
{
  ROSActionClientNodeRegister(std::string const& name)
  {
    ROS_WARN_STREAM(name << " ROS action client node registered");
    std::cout << name << " node registered" << "\n";
    BT::RegisterSimpleActionClient<T>(bt_factory_, name);
    ///ROS_WARN_STREAM(NodeRegister<T>::bt_factory_);
  }
};
