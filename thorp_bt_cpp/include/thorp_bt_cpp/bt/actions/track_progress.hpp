#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <thorp_toolkit/progress_tracker.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/** Track robot progress along a list of waypoints. */
class TrackProgress : public BT::StatefulActionNode
{
public:
  TrackProgress(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::PoseStamped>("robot_pose"),
             BT::InputPort<std::vector<geometry_msgs::PoseStamped>>("waypoints"),
             BT::InputPort<double>("reached_threshold"),
             BT::OutputPort<size_t>("next_waypoint") };
  }

private:
  BT::NodeStatus onStart() override
  {
    auto waypoints = getInput<std::vector<geometry_msgs::PoseStamped>>("waypoints");
    auto reached_threshold = getInput<double>("reached_threshold");
    pt_ = std::make_unique<ttk::ProgressTracker>(*waypoints, reached_threshold ? *reached_threshold : 1.0);
    ROS_INFO_STREAM_NAMED(name(), "Tracking progress along " << waypoints->size() << " waypoints");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    pt_->updatePose(*getInput<geometry_msgs::PoseStamped>("robot_pose"));
    setOutput("next_waypoint", pt_->nextWaypoint());
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    pt_->reset();
  }

  std::unique_ptr<ttk::ProgressTracker> pt_;
};
}  // namespace thorp::bt::actions
