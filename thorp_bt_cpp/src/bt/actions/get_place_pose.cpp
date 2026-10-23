#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <thorp_toolkit/geometry.hpp>

namespace thorp::bt::actions
{
/**
 * The pose that places the object the gripper holds at a position on a table: its center half its height above the
 * table's top, plus placing_height_on_table. Position and output on the table pose's frame; the yaw is left at zero,
 * as the arm turns the object toward where it reaches.
 */
class GetPlacePose : public BT::SyncActionNode
{
public:
  GetPlacePose(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("x"),                                    //
             BT::InputPort<double>("y"),                                    //
             BT::InputPort<moveit_msgs::msg::CollisionObject>("table"),     //
             BT::InputPort<geometry_msgs::msg::PoseStamped>("table_pose"),  //
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("place_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto x = requireInput<double>(*this, "x");
    const auto y = requireInput<double>(*this, "y");
    const auto table = requireInput<moveit_msgs::msg::CollisionObject>(*this, "table");
    const auto table_pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "table_pose");

    double z = table_pose.pose.position.z + objectHeight(table) / 2.0 +
               rosNode(*this)->get_parameter_or("placing_height_on_table", 0.005);
    // with nothing held, there is nothing to raise; the place action then fails, saying so
    const auto attached_objects = planningScene().getAttachedObjects();
    if (!attached_objects.empty())
      z += objectHeight(attached_objects.begin()->second.object) / 2.0;
    setOutput("place_pose", thorp::toolkit::createPose(x, y, z, 0.0, 0.0, 0.0, table_pose.header.frame_id));
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(GetPlacePose);
};

}  // namespace thorp::bt::actions
