#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/actions/action_client_node.hpp"

#include <mbf_msgs/GetPathAction.h>

namespace thorp::bt::actions
{
class GetPath : public BT::SimpleActionClientNode<mbf_msgs::GetPathAction>
{
public:
  GetPath(const std::string& name, const BT::NodeConfiguration& config) : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::SimpleActionClientNode<mbf_msgs::GetPathAction>::providedPorts();
    ports["action_name"].setDefaultValue("move_base_flex/get_path");
    ports.insert({ BT::InputPort<std::string>("planner"),
                   BT::InputPort<geometry_msgs::PoseStamped>("target_pose"),
                   BT::OutputPort<unsigned int>("error"),
                   BT::OutputPort<std::optional<mbf_msgs::GetPathFeedback>>("feedback") });
    return ports;
  }

  virtual void onFeedback(const mbf_msgs::GetPathFeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<mbf_msgs::GetPathFeedback>(*feedback));
  }

  virtual BT::NodeStatus onAborted(const mbf_msgs::GetPathResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace thorp::bt::actions
