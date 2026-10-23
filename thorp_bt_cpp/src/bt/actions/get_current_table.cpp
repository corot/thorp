#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/errors.hpp"
#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"
#include "thorp_bt_cpp/world.hpp"

#include <algorithm>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <thorp_msgs/msg/thorp_error.hpp>
#include <thorp_msgs/msg/world_entity.hpp>
#include <thorp_msgs/srv/query_world.hpp>

namespace thorp::bt::actions
{
using thorp_msgs::msg::ThorpError;

/**
 * The table the robot is at, as thorp_world tells: the one it docked at, or else the closest; its id, and the table
 * as GetTable gives it. Fails with NOT_AT_TABLE if it's at none.
 */
class GetCurrentTable : public BT::RosServiceNode<thorp_msgs::srv::QueryWorld, BT::SyncActionNode>
{
public:
  GetCurrentTable(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("world/query");
    ports.insert({ BT::OutputPort<std::string>("table_id"),                        //
                   BT::OutputPort<moveit_msgs::msg::CollisionObject>("table"),     //
                   BT::OutputPort<geometry_msgs::msg::PoseStamped>("table_pose"),  //
                   BT::OutputPort<ThorpError>("error") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.kind = thorp_msgs::msg::WorldEntity::TABLE;
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (response.at_table.empty())
    {
      const std::string text = "the robot is at no table" + (response.message.empty() ? "" : ": " + response.message);
      RCLCPP_WARN(logger(*this), "%s", text.c_str());
      setOutput("error", makeError(ThorpError::NOT_AT_TABLE, text));
      return BT::NodeStatus::FAILURE;
    }
    setOutput("table_id", response.at_table);
    const auto& entities = response.entities;
    const auto table =
        std::find_if(entities.begin(), entities.end(), [&](const auto& e) { return e.id == response.at_table; });
    if (table != entities.end())
    {
      setOutput("table", tableObject(*table));
      setOutput("table_pose", table->pose);
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailedRequest(FailureCause failure) override
  {
    setOutput("error", makeError(ThorpError::SERVER_NOT_AVAILABLE,
                                 "no answer from thorp_world's " +
                                     getInput<std::string>("service_name").value_or("query service")));
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(GetCurrentTable);
};
}  // namespace thorp::bt::actions
