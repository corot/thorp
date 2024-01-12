#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <geometry_msgs/PoseStamped.h>

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

  static NodeRegister<PushToList<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(PushToList<geometry_msgs::PoseStamped>, "PushPoseToList");
}  // namespace thorp::bt::actions
