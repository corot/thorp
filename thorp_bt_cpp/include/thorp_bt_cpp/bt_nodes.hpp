#pragma once

#include <ros/ros.h>

#include <behaviortree_cpp_v3/bt_factory.h>


namespace thorp::bt
{

void registerNodes(BT::BehaviorTreeFactory& bt_factory, ros::NodeHandle& nh, const std::string& nodes_file_path);

}  // namespace thorp::bt
