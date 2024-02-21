#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <thorp_msgs/PlanPickingAction.h>

namespace thorp::bt::actions
{
/**
 * Group objects into picking locations and sort them to make a picking plan
 */
class MakePickingPlan : public BT::RosActionNode<thorp_msgs::PlanPickingAction>
{
public:
  MakePickingPlan(const std::string& name, const BT::NodeConfiguration& config) : RosActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("manipulation/plan_picking");
    ports.insert({ BT::InputPort<geometry_msgs::PoseStamped>("robot_pose"),              //
                   BT::InputPort<std::vector<moveit_msgs::CollisionObject>>("objects"),  //
                   BT::InputPort<moveit_msgs::CollisionObject>("surface"),               //
                   BT::OutputPort<thorp_msgs::PickingPlan>("picking_plan") });
    return ports;
  }

private:
  std::optional<GoalType> getGoal() override
  {
    if (status() == BT::NodeStatus::RUNNING)
      return std::nullopt;

    GoalType goal;
    goal.robot_pose = *getInput<geometry_msgs::PoseStamped>("robot_pose");
    goal.objects = *getInput<std::vector<moveit_msgs::CollisionObject>>("objects");
    goal.surface = *getInput<moveit_msgs::CollisionObject>("surface");

    ros::NodeHandle pnh("~");
    pnh.getParam("picking_planning_frame", goal.planning_frame);
    pnh.getParam("approach_dist_to_table", goal.approach_dist);
    pnh.getParam("picking_dist_to_table", goal.picking_dist);
    pnh.getParam("detach_dist_from_table", goal.detach_dist);
    pnh.getParam("max_arm_reach", goal.max_arm_reach);
    goal.max_arm_reach -= pnh.param("tight_dist_tolerance", 0.0);
    
    return goal;
  }

  BT::NodeStatus onSucceeded(const ResultConstPtr& res) override
  {
    setOutput("picking_plan", res->picking_plan);

    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(MakePickingPlan);
};
}  // namespace thorp::bt::actions
