#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <geometry_msgs/PoseStamped.h>

namespace thorp::bt::actions
{
/**
 * Clear the content of a list.
 * @return BT::NodeStatus Returns FAILURE if no list is provided, SUCCESS otherwise.
 */
template <typename T>
class ClearList : public BT::SyncActionNode
{
public:
  ClearList(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::BidirectionalPort<std::vector<T>>("list") };
  }

private:
  BT::NodeStatus tick() override
  {
    if (auto list = getInput<std::vector<T>>("list"); list)
    {
      list->clear();
      setOutput("list", *list);
      return BT::NodeStatus::SUCCESS;
    }
    ROS_ERROR_STREAM_NAMED(name(), "No list provided");
    return BT::NodeStatus::FAILURE;
  }

  static NodeRegister<ClearList<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(ClearList<geometry_msgs::PoseStamped>, "ClearPoseList");
}  // namespace thorp::bt::actions
