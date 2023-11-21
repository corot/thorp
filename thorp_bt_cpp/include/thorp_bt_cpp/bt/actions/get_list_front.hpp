#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"

namespace thorp::bt::actions
{
/** Get the front element from a list. */
template <typename T>
class GetListFront : public BT::SyncActionNode
{
public:
  GetListFront(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& pnh)
    : BT::SyncActionNode(name, config), pnh_(pnh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<T>>("list"),
             BT::OutputPort<T>("element") };
  }

private:
  BT::NodeStatus tick() override
  {
    std::vector<T> list;
    utils::getInput<std::vector<T>>(*this, pnh_, "list", list);
    setOutput("element", list.front());
    
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle pnh_;
};
}  // namespace thorp::bt::actions
