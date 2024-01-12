#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <geometry_msgs/PoseStamped.h>

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

    {
      ROS_ERROR_STREAM("empty");
      return BT::NodeStatus::FAILURE;
    }
    setOutput("element", list.front());
    list.erase(list.begin());
    ROS_ERROR_STREAM(list.size());
    setOutput("list", list);
    return BT::NodeStatus::SUCCESS;
  }

  static NodeRegister<PopFromList<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(PopFromList<uint32_t>, "PopUIntFromList");
BT_REGISTER_TEMPLATE_NODE(PopFromList<geometry_msgs::PoseStamped>, "PopPoseFromList");
}  // namespace thorp::bt::actions
