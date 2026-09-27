#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <moveit_msgs/msg/collision_object.hpp>

#include <thorp_toolkit/tf2.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

#include <random>

namespace thorp::bt::actions
{
class SelectNextTarget : public BT::SyncActionNode
{
public:
  using Object = moveit_msgs::msg::CollisionObject;

  SelectNextTarget(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), tf2_(ttk::TF2::instance())
  {
    auto node = rosNode(*this);
    max_arm_reach_ = node->get_parameter_or("max_arm_reach", 0.3);
    max_failures_ = node->get_parameter_or("pickup_max_failures", 3);
    tightening_ = node->get_parameter_or("gripper_tightening", 0.002);
    std::string manip_frame = node->get_parameter_or<std::string>("pickup_planning_frame", "arm_base_link");
    arm_pose_on_bfp_rf_.header.frame_id = manip_frame;
    if (!tf2_.transformPose("base_footprint", arm_pose_on_bfp_rf_, arm_pose_on_bfp_rf_,
                             tf2::durationFromSec(10.0)))
      throw tf2::TransformException("Unable to get arm base pose on base_footprint reference frame");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<Object>>("objects"),               //
             BT::InputPort<std::map<std::string, uint32_t>>("failures"),  //
             BT::OutputPort<std::string>("target_name"),                  //
             BT::OutputPort<float>("tightening") };
  }

private:
  ttk::TF2& tf2_;
  uint32_t max_failures_;
  float max_arm_reach_;
  float tightening_;
  geometry_msgs::msg::PoseStamped arm_pose_on_bfp_rf_;

  BT::NodeStatus tick() override
  {
    auto objects = getInput<std::vector<Object>>("objects");
    if (!objects || objects->empty())
    {
      RCLCPP_INFO_STREAM(logger(*this), "No objects available");
      return BT::NodeStatus::FAILURE;
    }

    std::vector<std::tuple<std::string, double>> targets;

    for (const auto& obj : *objects)
    {
      // assuming objects are in arm base reference frame
      double dist = ttk::distance2D(obj.pose, arm_pose_on_bfp_rf_.pose);
      if (dist <= (max_arm_reach_ - 3e-3))  // 3 mm safety margin for perception noise
      {
        targets.emplace_back(obj.id, dist);
      }
    }

    if (targets.empty())
    {
      RCLCPP_WARN(logger(*this), "No targets within range (closer than %g m)", max_arm_reach_);
      return BT::NodeStatus::FAILURE;
    }

    // Sort by increasing distance
    std::sort(targets.begin(), targets.end(),
              [](const auto& a, const auto& b) { return std::get<1>(a) < std::get<1>(b); });

    auto failures = getInput<std::map<std::string, uint32_t>>("failures");
    auto failures_count = [&](std::string target)
    {
      if (!failures)
        return 0u;
      auto entry = failures->find(target);
      if (entry == failures->end())
        return 0u;
      return entry->second;
    };

    for (const auto& [target, dist] : targets)
    {
      if (auto fc = failures_count(target); fc < max_failures_)
      {
        if (fc)
        {
          static std::default_random_engine generator;
          std::uniform_real_distribution<float> uniform(0.0, tightening_ * 2.0 * fc);
          auto extra_tightening = uniform(generator);
          RCLCPP_INFO(logger(*this), "Retrying target '%s' (%d previous failures; %.1f mm of extra tightening)",
                         target.c_str(), fc, extra_tightening * 1000);
          setOutput("tightening", extra_tightening);
        }
        else
        {
          setOutput("tightening", tightening_);
        }
        setOutput("target_name", target);
        RCLCPP_INFO(logger(*this), "Next target: '%s', located at %.2f m from the arm", target.c_str(), dist);
        return BT::NodeStatus::SUCCESS;
      }
    }

    RCLCPP_WARN(logger(*this), "No targets to retry (failed less than %d times)", max_failures_);
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(SelectNextTarget);
};
}  // namespace thorp::bt::actions
