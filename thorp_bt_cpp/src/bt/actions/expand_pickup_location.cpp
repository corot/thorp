#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <thorp_msgs/msg/pickup_location.hpp>

namespace thorp::bt::actions
{
/**
 * The robot poses of a pickup location: approaching the table, picking and detached from it.
 */
class ExpandPickupLocation : public BT::SyncActionNode
{
public:
  ExpandPickupLocation(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<thorp_msgs::msg::PickupLocation>("pickup_location"),  //
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("approach_pose"),   //
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("pickup_pose"),     //
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("detach_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto pickup_location = requireInput<thorp_msgs::msg::PickupLocation>(*this, "pickup_location");
    setOutput("approach_pose", pickup_location.approach_pose);
    setOutput("pickup_pose", pickup_location.pickup_pose);
    setOutput("detach_pose", pickup_location.detach_pose);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ExpandPickupLocation);
};

}  // namespace thorp::bt::actions
