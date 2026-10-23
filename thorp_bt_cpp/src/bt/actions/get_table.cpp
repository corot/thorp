#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/errors.hpp"
#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"
#include "thorp_bt_cpp/world.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
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
                   BT::OutputPort<ThorpError>("error") });
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
      const std::string text = "thorp_world knows no table '" + tableId() + "'";
      RCLCPP_WARN(logger(*this), "%s", text.c_str());
      setOutput("error", makeError(ThorpError::TABLE_NOT_FOUND, text));
      return BT::NodeStatus::FAILURE;
    }
    const WorldEntity& entity = response.entities.front();
    setOutput("table", tableObject(entity));
    setOutput("table_pose", entity.pose);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailedRequest(FailureCause failure) override
  {
    setOutput("error", makeError(ThorpError::SERVER_NOT_AVAILABLE,
                                 "no answer from thorp_world's " +
                                     getInput<std::string>("service_name").value_or("query service")));
    return BT::NodeStatus::FAILURE;
  }

  std::string tableId() const
  {
    return getInput<std::string>("table_id").value_or("");
  }

  BT_REGISTER_NODE(GetTable);
};
}  // namespace thorp::bt::actions
