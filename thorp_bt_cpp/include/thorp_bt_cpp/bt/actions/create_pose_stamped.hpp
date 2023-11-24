#pragma once

#include <behaviortree_cpp_v3/action_node.h>

namespace thorp::bt::actions
{
class CreatePoseStamped : public BT::SyncActionNode
{
public:
  CreatePoseStamped(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& pnh)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("x"),
             BT::InputPort<double>("y"),
             BT::InputPort<double>("yaw"),
             BT::InputPort<std::string>("frame"),
             BT::OutputPort<geometry_msgs::PoseStamped>("pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    setOutput("pose", ttk::createPoseStamped(*getInput<double>("x"),
                                             *getInput<double>("y"),
                                             *getInput<double>("yaw"),
                                             *getInput<std::string>("frame")));
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace thorp::bt::actions
