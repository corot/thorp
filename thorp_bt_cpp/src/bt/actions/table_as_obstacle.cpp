#include <cmath>

#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/semantic_layer.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace thorp::bt::actions
{
/**
 * Mark a table as an obstacle on both costmaps, as the scans can't see its eaves.
 */
class TableAsObstacle : public BT::SyncActionNode
{
public:
  TableAsObstacle(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<moveit_msgs::msg::CollisionObject>("table"),  //
             BT::InputPort<geometry_msgs::msg::PoseStamped>("table_pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    using shape_msgs::msg::SolidPrimitive;
    const auto table = requireInput<moveit_msgs::msg::CollisionObject>(*this, "table");
    thorp_costmap_layers::msg::Object obstacle;
    obstacle.operation = thorp_costmap_layers::msg::Object::ADD;
    obstacle.type = "obstacle";
    obstacle.pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "table_pose");
    // detect_table names every table "table"; its position tells them apart
    obstacle.name = table.id + "_" + std::to_string(std::lround(obstacle.pose.pose.position.x)) + "_" +
                    std::to_string(std::lround(obstacle.pose.pose.position.y));
    obstacle.dimensions.x = table.primitives.at(0).dimensions.at(SolidPrimitive::BOX_X);
    obstacle.dimensions.y = table.primitives.at(0).dimensions.at(SolidPrimitive::BOX_Y);
    const bool marked = updateSemanticLayer(rosNode(*this), "local", { obstacle }) &&
                        updateSemanticLayer(rosNode(*this), "global", { obstacle });
    return marked ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(TableAsObstacle);
};

}  // namespace thorp::bt::actions
