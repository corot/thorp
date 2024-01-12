#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <geometry_msgs/PoseStamped.h>

namespace thorp::bt::actions
{
/**
 * Get the front element from a list.
 * @return BT::NodeStatus Returns FAILURE if the list is empty, SUCCESS otherwise.
 */
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
    std::vector<T> list = *getInput<std::vector<T>>("list");
    if (list.empty())
      return BT::NodeStatus::FAILURE;

    setOutput("element", list.front());
    return BT::NodeStatus::SUCCESS;
  }

  static NodeRegister<GetListFront<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(GetListFront<geometry_msgs::PoseStamped>, "GetPoseListFront");
}  // namespace thorp::bt::actions
