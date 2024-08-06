#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <thorp_msgs/MakePickupPlanAction.h>

namespace thorp::bt::actions
{
/**
 * Group objects into pickup locations and sort them to make a pickup plan
 */
class MakePickupPlan : public BT::RosActionNode<thorp_msgs::MakePickupPlanAction>
{
public:
  MakePickupPlan(const std::string& name, const BT::NodeConfig& config) : RosActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("manipulation/make_pickup_plan");
    ports.insert({ BT::InputPort<geometry_msgs::PoseStamped>("robot_pose"),                 //
                   BT::InputPort<std::vector<geometry_msgs::PoseStamped>>("pickup_poses"),  //
                   BT::InputPort<std::vector<moveit_msgs::CollisionObject>>("objects"),     //
                   BT::InputPort<moveit_msgs::CollisionObject>("surface"),                  //
                   BT::OutputPort<std::vector<thorp_msgs::PickupLocation>>("pickup_plan") });
    return ports;
  }

private:
  std::optional<GoalType> getGoal() override
  {
    if (status() == BT::NodeStatus::RUNNING)
      return std::nullopt;

    GoalType goal;
    goal.robot_pose = *getInput<geometry_msgs::PoseStamped>("robot_pose");
    goal.pickup_poses = *getInput<std::vector<geometry_msgs::PoseStamped>>("pickup_poses");
    goal.objects = *getInput<std::vector<moveit_msgs::CollisionObject>>("objects");
    goal.surface = *getInput<moveit_msgs::CollisionObject>("surface");

    ros::NodeHandle pnh("~");
    pnh.getParam("pickup_planning_frame", goal.planning_frame);
    pnh.getParam("approach_dist_to_table", goal.approach_dist);
    pnh.getParam("pickup_dist_to_table", goal.pickup_dist);
    pnh.getParam("detach_dist_from_table", goal.detach_dist);
    pnh.getParam("max_arm_reach", goal.max_arm_reach);
    goal.max_arm_reach -= pnh.param("tight_dist_tolerance", 0.0);
    
    return goal;
  }

  BT::NodeStatus onSucceeded(const ResultConstPtr& res) override
  {
    setOutput("pickup_plan", res->pickup_plan.locations);

    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(MakePickupPlan);
};
}  // namespace thorp::bt::actions
