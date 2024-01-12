#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/progress_tracker.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/** Track robot progress along a list of waypoints. */
class TrackProgress : public BT::StatefulActionNode
{
public:
  TrackProgress(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::PoseStamped>("robot_pose"), BT::BidirectionalPort<Waypoints>("waypoints"),
             BT::InputPort<double>("reached_threshold"), BT::OutputPort<size_t>("next_waypoint") };
  }

private:
  using Waypoints = std::vector<geometry_msgs::PoseStamped>;

  BT::NodeStatus onStart() override
  {
    next_waypoint_ = 0;
    waypoints_ = *getInput<Waypoints>("waypoints");
    auto reached_threshold = getInput<double>("reached_threshold");
    pt_ = std::make_unique<ttk::ProgressTracker>(waypoints_, reached_threshold ? *reached_threshold : 1.0);
    ROS_INFO_STREAM_NAMED(name(), "Tracking progress along " << waypoints_.size() << " waypoints");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    pt_->updatePose(*getInput<geometry_msgs::PoseStamped>("robot_pose"));
    if (next_waypoint_ != pt_->nextWaypoint())
    {
      next_waypoint_ = pt_->nextWaypoint();
      setOutput("waypoints",
                Waypoints{ waypoints_.begin() + std::min(next_waypoint_, waypoints_.size()), waypoints_.end() });
      setOutput("next_waypoint", next_waypoint_);
      auto v = Waypoints{ waypoints_.begin() + next_waypoint_, waypoints_.end() };
      ROS_INFO_STREAM_NAMED(name(), "Next waypoint: " << next_waypoint_ << " (" << v.size() << "/" << waypoints_.size()
                                                      << " left)");
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    pt_->reset();
  }

  std::unique_ptr<ttk::ProgressTracker> pt_;
  size_t next_waypoint_;
  Waypoints waypoints_;

  BT_REGISTER_NODE(TrackProgress);
};
}  // namespace thorp::bt::actions
