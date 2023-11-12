#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"

namespace thorp::bt::actions
{
/** Pop and return an element from the beginning of a list. */
template <typename T>
class PopFromList : public BT::SyncActionNode
{
public:
  PopFromList(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& pnh)
    : BT::SyncActionNode(name, config), pnh_(pnh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::BidirectionalPort<std::vector<T>>("list"),
             BT::OutputPort<T>("element") };
  }

private:
  BT::NodeStatus tick() override
  {
    std::vector<T> list;
    utils::getInput<std::vector<T>>(*this, pnh_, "list", list);
    setOutput("element", list.front());
    list.erase(list.begin());
    setOutput("list", list);
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle pnh_;
};
}  // namespace thorp::bt::actions
