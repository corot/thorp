#include <mutex>
#include <optional>

#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Wait for the target tracker to see a target, publishing its pose. Only poses received after starting count.
 * @return SUCCESS with the target pose if one comes before the timeout, FAILURE otherwise
 */
class MonitorTarget : public BT::StatefulActionNode
{
public:
  MonitorTarget(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("topic_name", "target_object_pose", "Target pose topic"),
             BT::InputPort<double>("timeout", 0.0, "Seconds to wait for a target; forever if <= 0"),
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose") };
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::mutex mutex_;  // bt_server receives messages on another thread than the one ticking the tree
  std::optional<geometry_msgs::msg::PoseStamped> target_pose_;
  rclcpp::Time start_time_;

  BT::NodeStatus onStart() override
  {
    if (!sub_)
    {
      sub_ = rosNode(*this)->create_subscription<geometry_msgs::msg::PoseStamped>(
          requireInput<std::string>(*this, "topic_name"), 1,
          [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            target_pose_ = *msg;
          });
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target_pose_.reset();
    }
    start_time_ = rosNode(*this)->now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::optional<geometry_msgs::msg::PoseStamped> target_pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target_pose.swap(target_pose_);
    }
    if (target_pose)
    {
      RCLCPP_INFO_THROTTLE(logger(*this), *rosNode(*this)->get_clock(), 5000, "Target seen at %s",
                           ttk::toStr2D(*target_pose).c_str());
      setOutput("target_pose", *target_pose);
      return BT::NodeStatus::SUCCESS;
    }
    const double timeout = requireInput<double>(*this, "timeout");
    if (timeout > 0.0 && (rosNode(*this)->now() - start_time_).seconds() > timeout)
    {
      RCLCPP_DEBUG(logger(*this), "No target seen in %g seconds", timeout);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
  }

  BT_REGISTER_NODE(MonitorTarget);
};
}  // namespace thorp::bt::actions
