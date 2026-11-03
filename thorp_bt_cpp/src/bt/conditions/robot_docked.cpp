#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_msgs/msg/world_entity.hpp>
#include <thorp_msgs/srv/query_world.hpp>

namespace thorp::bt::conditions
{
/**
 * Check whether the robot is docked at a table, as thorp_world tells: it docked there and is still at it. Returns:
 * SUCCESS if so, with the table's id
 * FAILURE otherwise, or if the world can't tell
 */
class RobotDocked : public BT::RosServiceNode<thorp_msgs::srv::QueryWorld, BT::ConditionNode>
{
public:
  RobotDocked(const std::string& name, const BT::NodeConfig& conf) : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("world/query");
    ports.insert({ BT::OutputPort<std::string>("table_id") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.kind = thorp_msgs::msg::WorldEntity::TABLE;
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (response.docked_at.empty())
      return BT::NodeStatus::FAILURE;
    setOutput("table_id", response.docked_at);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(RobotDocked);
};
}  // namespace thorp::bt::conditions
