#pragma once

#include "thorp_bt_cpp/bt_node_register.hpp"

#include <behaviortree_cpp_v3/action_node.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class CreatePoseStamped : public BT::SyncActionNode
{
public:
  CreatePoseStamped(const std::string& name, const BT::NodeConfiguration& config)
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

  static NodeRegister<CreatePoseStamped> reg;
};
}  // namespace thorp::bt::actions


NodeRegister<thorp::bt::actions::CreatePoseStamped> thorp::bt::actions::CreatePoseStamped::reg("CreatePoseStamped");
