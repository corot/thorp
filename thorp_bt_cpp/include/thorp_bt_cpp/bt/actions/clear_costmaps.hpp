#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/actions/service_client_node.hpp"

#include <std_srvs/Empty.h>

namespace thorp::bt::actions
{
class ClearCostmaps : public BT::RosServiceNode<std_srvs::Empty>
{
public:
  ClearCostmaps(ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode<std_srvs::Empty>(handle, node_name, conf)
  {
  }

  void sendRequest(RequestType& request) override
  {
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace thorp::bt::actions
