#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <thorp_msgs/PickingPlan.h>

namespace thorp::bt::actions
{
/**
 * Pop a location from the picking plan and extract all fields on separated keys.
 * @return BT::NodeStatus Returns FAILURE if the picking plan is empty, SUCCESS otherwise.
 */
class PopPickupLocation : public BT::SyncActionNode
{
public:
  PopPickupLocation(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::BidirectionalPort<thorp_msgs::PickingPlan>("picking_plan"),  //
             BT::OutputPort<geometry_msgs::PoseStamped>("approach_pose"),     //
             BT::OutputPort<geometry_msgs::PoseStamped>("picking_pose"),      //
             BT::OutputPort<geometry_msgs::PoseStamped>("detach_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    auto picking_plan = *getInput<thorp_msgs::PickingPlan>("picking_plan");
    if (picking_plan.locations.empty())
    {
      ROS_INFO_NAMED(name(), "Picking plan is empty");
      return BT::NodeStatus::FAILURE;
    }
    auto picking_loc = picking_plan.locations.front();
    picking_plan.locations.erase(picking_plan.locations.begin());
    setOutput("approach_pose", picking_loc.approach_pose);
    setOutput("picking_pose", picking_loc.picking_pose);
    setOutput("detach_pose", picking_loc.detach_pose);
    setOutput("picking_plan", picking_plan);
    return BT::NodeStatus::SUCCESS;
  }

    BT_REGISTER_NODE(PopPickupLocation);
};
}  // namespace thorp::bt::actions
