#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"

namespace thorp::bt::actions
{
/** Insert an element at the beginning of a list. */
template <typename T>
class PushToList : public BT::SyncActionNode
{
public:
  PushToList(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& pnh)
    : BT::SyncActionNode(name, config), pnh_(pnh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::BidirectionalPort<std::vector<T>>("list"),
             BT::InputPort<T>("element") };
  }

private:
  BT::NodeStatus tick() override
  {
    std::vector<T> list;
    utils::getInput<std::vector<T>>(*this, pnh_, "list", list);
    ROS_ERROR_STREAM(utils::getInput<T>(*this, pnh_, "element"));
    list.insert(list.begin(), utils::getInput<T>(*this, pnh_, "element"));
    setOutput("list", list);
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle pnh_;
};
}  // namespace thorp::bt::actions
