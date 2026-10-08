#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <rclcpp/time.hpp>
#include <thorp_msgs/msg/world_entity.hpp>
#include <thorp_msgs/srv/query_world.hpp>

namespace thorp::bt::conditions
{
using thorp_msgs::msg::WorldEntity;

/**
 * Check whether a table has been visited already: thorp_world says the robot looked it over, as validate_table
 * reports. A world loaded from a file remembers the tables inspected before. Returns:
 * SUCCESS if so
 * FAILURE otherwise, or if the world doesn't know the table
 */
class TableVisited : public BT::RosServiceNode<thorp_msgs::srv::QueryWorld, BT::ConditionNode>
{
public:
  TableVisited(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("world/query");
    ports.insert({ BT::InputPort<std::string>("table_id") });
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
    const std::string table_id = getInput<std::string>("table_id").value_or("");
    if (response.entities.empty())
    {
      RCLCPP_WARN(logger(*this), "No table '%s'", table_id.c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (rclcpp::Time(response.entities.front().inspected).nanoseconds() == 0)
    {
      RCLCPP_INFO(logger(*this), "New table '%s'", table_id.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger(*this), "Table '%s' already visited", table_id.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(TableVisited);
};
}  // namespace thorp::bt::conditions
