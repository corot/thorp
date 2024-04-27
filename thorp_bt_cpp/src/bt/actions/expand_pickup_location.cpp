#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <thorp_msgs/PickLocation.h>

namespace thorp::bt::actions
{
/**
 * Extract all fields from a picking plan location into separated keys.
 * @return Always SUCCESS.
 */
class ExpandPickupLocation : public BT::SyncActionNode
{
public:
  ExpandPickupLocation(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<thorp_msgs::PickLocation>("pickup_location"),   //
             BT::OutputPort<geometry_msgs::PoseStamped>("approach_pose"),  //
             BT::OutputPort<geometry_msgs::PoseStamped>("picking_pose"),   //
             BT::OutputPort<geometry_msgs::PoseStamped>("detach_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    auto pickup_location = *getInput<thorp_msgs::PickLocation>("pickup_location");
    setOutput("approach_pose", pickup_location.approach_pose);
    setOutput("picking_pose", pickup_location.picking_pose);
    setOutput("detach_pose", pickup_location.detach_pose);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ExpandPickupLocation);
};
}  // namespace thorp::bt::actions
