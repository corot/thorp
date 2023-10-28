#pragma once

#include <ros/ros.h>

#include <behaviortree_cpp_v3/bt_factory.h>

#include "thorp_bt_cpp/bt/utils.hpp"


namespace thorp::bt
{

/**
 * Helper function to register a BT node.
 */
template <typename T, typename... Args>
void registerNode(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh, const std::string& name, Args&&... args)
{
  bt::utils::registerNode<T, Args...>(factory, nh, name, std::forward<Args>(args)...);
}

void registerNodes(BT::BehaviorTreeFactory& bt_factory, ros::NodeHandle& nh, const std::string& nodes_file_path);

}  // namespace thorp::bt
