#include <behaviortree_cpp_v3/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::conditions
{
/**
 * Check whether two poses are the same (within tolerance thresholds)
 */
class AreSamePose : public BT::ConditionNode
{
public:
  AreSamePose(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("dist_tolerance"),             //
             BT::InputPort<double>("angle_tolerance"),            //
             BT::InputPort<geometry_msgs::PoseStamped>("pose1"),  //
             BT::InputPort<geometry_msgs::PoseStamped>("pose2") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto dist_tolerance = getInput<double>("dist_tolerance");
    const auto angle_tolerance = getInput<double>("angle_tolerance");
    const auto pose1 = *getInput<geometry_msgs::PoseStamped>("pose1");
    const auto pose2 = *getInput<geometry_msgs::PoseStamped>("pose2");
    const auto dist_t = dist_tolerance ? *dist_tolerance : 0.1;
    const auto angle_t = angle_tolerance ? *angle_tolerance : 0.1;
    if (ttk::samePose(pose1, pose2, dist_t, angle_t))
    {
      ROS_INFO_NAMED(name(), "Same pose: %s == %s with %f, %f tolerances", ttk::toCStr2D(pose1), ttk::toCStr2D(pose2),
                     dist_t, angle_t);
      return BT::NodeStatus::SUCCESS;
    }
    ROS_INFO_NAMED(name(), "Diff pose: %s != %s with %f, %f tolerances", ttk::toCStr2D(pose1), ttk::toCStr2D(pose2),
                   dist_t, angle_t);
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(AreSamePose);
};
}  // namespace thorp::bt::conditions
