#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"

#include <std_srvs/Empty.h>

namespace thorp::bt::actions
{
// TODO ServiceNode from BehaviorTreeROS
class ClearCostmaps : public BT::SyncActionNode
{
public:
  ClearCostmaps(const std::string& name)
    : SyncActionNode(name, {})
    , client_(ros::NodeHandle().serviceClient<std_srvs::Empty>("move_base_flex/clear_costmaps"))
  {
    while (ros::ok() && !client_.waitForExistence(ros::Duration(0.5)))
    {
      // ros::ServiceClient::waitForExistence already prints logs
      ros::spinOnce();
    }
  }

  BT::NodeStatus tick() override
  {
    std_srvs::Empty srv;
    return client_.call(srv) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ros::ServiceClient client_;
};
}  // namespace thorp::bt::actions
