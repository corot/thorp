#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/semantic_layer.hpp"

#include <moveit_msgs/msg/collision_object.hpp>

namespace thorp::bt::actions
{
/**
 * Clear an area around a pickup pose on the local costmap, so the robot can dock at the table, under its eaves.
 */
class ClearTableAccess : public BT::SyncActionNode
{
public:
  ClearTableAccess(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<moveit_msgs::msg::CollisionObject>("table"),  //
             BT::InputPort<geometry_msgs::msg::PoseStamped>("table_pose", "pickup pose at the table") };
  }

private:
  BT::NodeStatus tick() override
  {
    thorp_costmap_layers::msg::Object access;
    access.operation = thorp_costmap_layers::msg::Object::ADD;
    access.type = "free_space";
    access.name = requireInput<moveit_msgs::msg::CollisionObject>(*this, "table").id + " approach";
    access.pose = requireInput<geometry_msgs::msg::PoseStamped>(*this, "table_pose");
    access.dimensions.x = 1.0;
    access.dimensions.y = 0.5;
    return updateSemanticLayer(rosNode(*this), "local", { access }) ? BT::NodeStatus::SUCCESS :
                                                                      BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(ClearTableAccess);
};

}  // namespace thorp::bt::actions
