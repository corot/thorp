#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <thorp_msgs/msg/thorp_error.hpp>
#include <thorp_msgs/msg/world_entity.hpp>
#include <thorp_msgs/srv/query_world.hpp>

namespace thorp::bt::actions
{
using moveit_msgs::msg::CollisionObject;
using thorp_msgs::msg::ThorpError;
using thorp_msgs::msg::WorldEntity;

/**
 * A table thorp_world knows, as the table nodes take it: a box of its size, and its pose, on the map frame.
 */
class GetTable : public BT::RosServiceNode<thorp_msgs::srv::QueryWorld, BT::SyncActionNode>
{
public:
  GetTable(const std::string& name, const BT::NodeConfig& conf) : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("world/query");
    ports.insert({ BT::InputPort<std::string>("table_id"),                         //
                   BT::OutputPort<CollisionObject>("table"),                       //
                   BT::OutputPort<geometry_msgs::msg::PoseStamped>("table_pose"),  //
                   BT::OutputPort<int>("error") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.ids = { requireInput<std::string>(*this, "table_id") };
    request.kind = WorldEntity::TABLE;
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (response.entities.empty())
    {
      RCLCPP_WARN(logger(*this), "No table '%s'", tableId().c_str());
      setOutput("error", static_cast<int>(ThorpError::TABLE_NOT_FOUND));
      return BT::NodeStatus::FAILURE;
    }
    const WorldEntity& entity = response.entities.front();
    CollisionObject table;
    table.id = entity.id;
    table.header = entity.pose.header;
    table.pose = entity.pose.pose;
    table.operation = CollisionObject::ADD;
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = { entity.size.x, entity.size.y, entity.size.z };
    table.primitive_poses.resize(1);
    table.primitive_poses[0].orientation.w = 1.0;
    // as detections describe tables
    table.type.db =
        nlohmann::json{ { "size", { entity.size.x, entity.size.y, entity.size.z } }, { "color", entity.color } }.dump();
    setOutput("table", table);
    setOutput("table_pose", entity.pose);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailedRequest(FailureCause failure) override
  {
    setOutput("error", static_cast<int>(ThorpError::SERVER_NOT_AVAILABLE));
    return BT::NodeStatus::FAILURE;
  }

  std::string tableId() const
  {
    return getInput<std::string>("table_id").value_or("");
  }

  BT_REGISTER_NODE(GetTable);
};
}  // namespace thorp::bt::actions
