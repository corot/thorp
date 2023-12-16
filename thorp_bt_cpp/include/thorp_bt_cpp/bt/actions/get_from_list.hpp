#pragma once

#include <behaviortree_cpp_v3/action_node.h>

namespace thorp::bt::actions
{
/**
 * Get an element from a list at a given index, optionally popping the element.
 * @return BT::NodeStatus Returns FAILURE if the list is empty, SUCCESS otherwise.
 */
template <typename T>
class GetFromList : public BT::SyncActionNode
{
public:
  GetFromList(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<bool>("pop", false, "Remove the returned element"),
             BT::InputPort<int32_t>("index", 0, "Index of the element to return; negative values count from the end"),
             BT::BidirectionalPort<std::vector<T>>("list"),
             BT::OutputPort<T>("element") };
  }

private:
  BT::NodeStatus tick() override
  {
    std::vector<T> list = *getInput<std::vector<T>>("list");
    if (list.empty())
      return BT::NodeStatus::FAILURE;

    int32_t index = *getInput<int32_t>("index");
    if (index < 0)
      index += list.size();
    setOutput("element", list[index]);

    auto pop = getInput<bool>("pop");
    if (pop && *pop)
    {
      list.erase(list.begin() + index);
      setOutput("list", list);
    }
    ROS_DEBUG_STREAM(name() << "\tindex: " << index << "\telement: " << list[index] << (pop ? "\tpop" : ""));
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace thorp::bt::actions
