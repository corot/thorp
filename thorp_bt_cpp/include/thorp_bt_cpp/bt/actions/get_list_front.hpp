#pragma once

#include <behaviortree_cpp_v3/action_node.h>

namespace thorp::bt::actions
{
/** Get the front element from a list. */
template <typename T>
class GetListFront : public BT::SyncActionNode
{
public:
  GetListFront(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
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
    getInput<std::vector<T>>("list", list);
    setOutput("element", list.front());
    
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace thorp::bt::actions
