#pragma once

#include <behaviortree_cpp_v3/action_node.h>

namespace thorp::bt::actions
{
/**
 * Pop and return an element from the beginning of a list.
 * @return BT::NodeStatus Returns FAILURE if the list is empty, SUCCESS otherwise.
 */
template <typename T>
class PopFromList : public BT::SyncActionNode
{
public:
  PopFromList(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
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
    std::vector<T> list = *getInput<std::vector<T>>("list");
    if (list.empty())
      return BT::NodeStatus::FAILURE;

    setOutput("element", list.front());
    list.erase(list.begin());
    setOutput("list", list);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace thorp::bt::actions
