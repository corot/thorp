#include <cmath>

#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/service_call.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_msgs/srv/get_cost.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Poses facing the table at a distance from each of its sides, one per side, or two on sides longer than twice the
 * arm reach if split_long_sides; none on sides shorter than table_min_pickup_side. Poses on obstacles on the global
 * costmap are discarded; those on the table's inflation are not, as pickup poses are under its eaves.
 */
class GetPosesAroundTable : public BT::SyncActionNode
{
public:
  GetPosesAroundTable(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
    auto node = rosNode(*this);
    max_arm_reach_ = node->get_parameter_or("max_arm_reach", 0.3);
    min_pickup_side_ = node->get_parameter_or("table_min_pickup_side", 0.35);
    auto qos = rclcpp::QoS(1).transient_local();
    valid_poses_pub_ = node->create_publisher<geometry_msgs::msg::PoseArray>("~/valid_poses_around_table", qos);
    blocked_poses_pub_ = node->create_publisher<geometry_msgs::msg::PoseArray>("~/blocked_poses_around_table", qos);
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<moveit_msgs::msg::CollisionObject>("table"),     //
             BT::InputPort<geometry_msgs::msg::PoseStamped>("table_pose"),  //
             BT::InputPort<double>("distance"),                             //
             BT::InputPort<bool>("split_long_sides", false, "Create two poses on sides longer than max_arm_reach x 2"),
             BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("table_side_poses") };
  }

private:
  double max_arm_reach_;
  double min_pickup_side_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr valid_poses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr blocked_poses_pub_;

  BT::NodeStatus tick() override
  {
    using shape_msgs::msg::SolidPrimitive;
    const auto table = requireInput<moveit_msgs::msg::CollisionObject>(*this, "table");
    const auto table_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "table_pose");
    const auto split_long = requireInput<bool>(*this, "split_long_sides");
    const auto distance = requireInput<double>(*this, "distance");
    const double depth = table.primitives.at(0).dimensions.at(SolidPrimitive::BOX_X);
    const double width = table.primitives.at(0).dimensions.at(SolidPrimitive::BOX_Y);

    // 0, 1 or 2 poses for each of the four sides around the table, on the table frame
    const double p_x = distance + depth / 2.0;
    const double p_y = distance + width / 2.0;
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    auto add_poses = [&](double x, double y, double yaw, double side) {
      if (side < min_pickup_side_)
        return;  // too narrow to pickup
      if (side < max_arm_reach_ * 2.0 || !split_long)
      {
        poses.push_back(ttk::createPose(x, y, yaw, "table_frame"));
        return;
      }
      for (const double half : { -side / 4.0, side / 4.0 })
        poses.push_back(ttk::createPose(x == 0.0 ? half : x, y == 0.0 ? half : y, yaw, "table_frame"));
    };
    add_poses(p_x, 0.0, -M_PI, width);
    add_poses(-p_x, 0.0, 0.0, width);
    add_poses(0.0, p_y, -M_PI / 2.0, depth);
    add_poses(0.0, -p_y, M_PI / 2.0, depth);

    // to the table pose's frame, at floor level
    auto table_tf = ttk::toTransform(table_pose);
    table_tf.transform.translation.z = 0.0;
    for (auto& pose : poses)
    {
      tf2::doTransform(pose, pose, table_tf);
      pose.header.frame_id = table_tf.header.frame_id;
    }

    std::vector<geometry_msgs::msg::PoseStamped> valid, blocked;
    for (const auto& pose : poses)
      (isBlocked(pose) ? blocked : valid).push_back(pose);
    RCLCPP_INFO_EXPRESSION(logger(*this), !blocked.empty(), "%zu poses out of %zu discarded as blocked",
                           blocked.size(), poses.size());
    RCLCPP_WARN_EXPRESSION(logger(*this), valid.empty(), "Every pose around the table is blocked; returning none");
    valid_poses_pub_->publish(ttk::toPoseArray(valid, 0.01, table_tf.header.frame_id));
    blocked_poses_pub_->publish(ttk::toPoseArray(blocked, 0.01, table_tf.header.frame_id));
    setOutput("table_side_poses", valid);
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * Whether the pose is on an obstacle on the global costmap; not blocked if the costmap can't tell
   */
  bool isBlocked(const geometry_msgs::msg::PoseStamped& pose)
  {
    geometry_msgs::msg::PoseStamped map_pose;
    if (!ttk::TF2::instance().transformPose("map", pose, map_pose))
      return false;
    nav2_msgs::srv::GetCost::Request request;
    request.use_footprint = false;
    request.x = map_pose.pose.position.x;
    request.y = map_pose.pose.position.y;
    request.theta = ttk::yaw(map_pose);
    const auto response =
        callService<nav2_msgs::srv::GetCost>(rosNode(*this), "/global_costmap/get_cost_global_costmap", request);
    if (!response)
      return false;
    RCLCPP_DEBUG(logger(*this), "Pose [%.2f, %.2f] cost: %g", request.x, request.y, response->cost);
    return response->cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  BT_REGISTER_NODE(GetPosesAroundTable);
};

}  // namespace thorp::bt::actions
