#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <geometry_msgs/PoseStamped.h>

namespace thorp::bt::actions
{
/** Crop elements from the beginning and end of a list. */
template <typename T>
class ListSlicing : public BT::SyncActionNode
{
public:
  ListSlicing(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::BidirectionalPort<std::vector<T>>("list"), BT::InputPort<size_t>("start"),
             BT::InputPort<size_t>("end") };
  }

private:
  BT::NodeStatus tick() override
  {
    auto list = getInput<std::vector<T>>("list");
    if (!list)
    {
      ROS_ERROR_STREAM_NAMED(name(), "No list provided");
      return BT::NodeStatus::FAILURE;
    }
    auto opt_start = getInput<size_t>("start");
    auto opt_end = getInput<size_t>("end");
    int start = opt_start ? *opt_start : 0;
    int end = opt_end ? *opt_end : 0;
    auto new_list = std::vector<T>(list->begin() + start, list->end() + end);
    ROS_INFO_NAMED(name(), "Slicing list[%d:%d] (size %lu -> %lu)", start, end, list->size(), new_list.size());
    setOutput("list", new_list);
    return BT::NodeStatus::SUCCESS;
  }

  static NodeRegister<ListSlicing<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(ListSlicing<geometry_msgs::PoseStamped>, "PoseListSlicing");
}  // namespace thorp::bt::actions
