#include "thorp_bt_cpp/ros_action_node.hpp"

#include "thorp_bt_cpp/node_common.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <thorp_msgs/action/detect_tables.hpp>

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Detect the table in front of the robot, if its shortest side reaches the table_min_side parameter.
 * The table is a box collision object, with x along its longest side; its pose is also provided on the map frame.
 */
class DetectTables : public BT::RosActionNode<thorp_msgs::action::DetectTables>
{
public:
  DetectTables(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : RosActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::OutputPort<moveit_msgs::msg::CollisionObject>("table"),     //
                                BT::OutputPort<geometry_msgs::msg::PoseStamped>("table_pose"),  //
                                BT::OutputPort<std::string>("table_id", "as thorp_world calls it") });
  }

private:
  Goal getGoal() override
  {
    Goal goal;
    goal.min_side = node_->get_parameter_or("table_min_side", 0.3);
    return goal;
  }

  BT::NodeStatus onSucceeded(const Result& result) override
  {
    if (result.tables.empty())
    {
      // No tables detected
      return BT::NodeStatus::FAILURE;
    }

    const auto& table = result.tables.front();
    geometry_msgs::msg::PoseStamped table_pose;
    table_pose.header = table.header;
    table_pose.pose = table.pose;
    if (!ttk::TF2::instance().transformPose("map", table_pose, table_pose))
    {
      // Transform table pose to map frame failed
      return BT::NodeStatus::FAILURE;
    }

    const auto& size = table.primitives.front().dimensions;
    RCLCPP_INFO(logger(*this), "Detected table of size %.1f x %.1f at %s", size[1], size[0],
                ttk::toCStr2D(table_pose));
    setOutput("table", table);
    setOutput("table_pose", table_pose);
    setOutput("table_id", table.id);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onAborted(const Result& /*result*/) override
  {
    RCLCPP_ERROR(logger(*this), "Table detection failed");
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_ACTION_NODE(DetectTables, "object_detection/detect_tables");
};

}  // namespace thorp::bt::actions
