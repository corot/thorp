#pragma once

#include <behaviortree_cpp_v3/action_node.h>

namespace thorp::bt::actions
{
/** Insert an element at the beginning of a list. */
template <typename T>
class PushToList : public BT::SyncActionNode
{
public:
  PushToList(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
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
    getInput<std::vector<T>>("list", list);
    list.insert(list.begin(), *getInput<T>("element"));
    setOutput("list", list);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace thorp::bt::actions
