#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <rail_manipulation_msgs/SegmentedObject.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Calculate the four locations around a table at a given distance and return the closest to the robot.
 */
class ClosestTableSide : public BT::SyncActionNode
{
public:
  ClosestTableSide(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<rail_manipulation_msgs::SegmentedObject>("table"),  //
             BT::InputPort<geometry_msgs::PoseStamped>("table_pose"),          //
             BT::InputPort<geometry_msgs::PoseStamped>("robot_pose"),          //
             BT::InputPort<double>("distance"),                                //
             BT::OutputPort<geometry_msgs::PoseStamped>("closest_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto table = *getInput<rail_manipulation_msgs::SegmentedObject>("table");
    const auto table_pose = *getInput<geometry_msgs::PoseStamped>("table_pose");
    const auto robot_pose = *getInput<geometry_msgs::PoseStamped>("robot_pose");
    const auto distance = *getInput<double>("distance");

    // Create poses for the four sides around the table
    const double p_x = distance + table.depth / 2.0;
    const double n_x = -p_x;
    const double p_y = distance + table.width / 2.0;
    const double n_y = -p_y;

    std::vector<geometry_msgs::PoseStamped> poses = { ttk::createPose(p_x, 0.0, -M_PI, "table_frame"),
                                                      ttk::createPose(n_x, 0.0, 0.0, "table_frame"),
                                                      ttk::createPose(0.0, p_y, -M_PI / 2.0, "table_frame"),
                                                      ttk::createPose(0.0, n_y, M_PI / 2.0, "table_frame") };
    const auto table_tf = ttk::toTransform(table_pose);

    // Find the closest pose to the robot
    geometry_msgs::PoseStamped closest_pose;
    double closest_dist = std::numeric_limits<double>::infinity();
    for (auto& pose : poses)
    {
      // We need to transform table side poses to robot pose frame, as distance2D assumes same frame
      tf2::doTransform(pose, pose, table_tf);
      double dist = ttk::distance2D(pose, robot_pose);
      if (dist < closest_dist)
      {
        closest_pose = pose;
        closest_dist = dist;
      }
    }
    setOutput("closest_pose", closest_pose);

    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ClosestTableSide);
};
}  // namespace thorp::bt::actions