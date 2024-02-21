#include <behaviortree_cpp_v3/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <thorp_msgs/PickLocation.h>

namespace thorp::bt::conditions
{
/**
 * Checks if the given list is empty.
 *
 * @param[in] list List to evaluate
 *
 * @return  SUCCESS if the given list doesn't exist or it's empty
 *          FAILURE if the given list is not empty
 */
template <typename T>
class IsListEmpty : public BT::ConditionNode
{
public:
  IsListEmpty(const std::string& name, const BT::NodeConfiguration& config)
    : ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<T>>("list") };
  }

  BT::NodeStatus tick() override
  {
    auto list = getInput<std::vector<T>>("list");
    return !list || list->empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  static NodeRegister<IsListEmpty<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(IsListEmpty<uint32_t>, "IsUIntListEmpty");
BT_REGISTER_TEMPLATE_NODE(IsListEmpty<geometry_msgs::PoseStamped>, "IsPoseListEmpty");
BT_REGISTER_TEMPLATE_NODE(IsListEmpty<thorp_msgs::PickLocation>, "PickupPlanCompleted");
}  // namespace thorp::bt::conditions
