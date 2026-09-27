#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

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
  PopFromList(const std::string& name, const BT::NodeConfig& config)
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
    std::vector<T> list = requireInput<std::vector<T>>(*this, "list");
    if (list.empty())
    {
      RCLCPP_DEBUG_STREAM(logger(*this), "Tried to pop from empty list");
      return BT::NodeStatus::FAILURE;
    }
    setOutput("element", list.front());
    list.erase(list.begin());
    setOutput("list", list);
    RCLCPP_DEBUG_STREAM(logger(*this), ":\tnew size is " << list.size());
    return BT::NodeStatus::SUCCESS;
  }

  static NodeRegister<PopFromList<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(PopFromList<uint32_t>, "PopUIntFromList");
BT_REGISTER_TEMPLATE_NODE(PopFromList<geometry_msgs::msg::PoseStamped>, "PopPoseFromList");
}  // namespace thorp::bt::actions
