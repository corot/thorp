#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_msgs/msg/thorp_error.hpp>
#include <thorp_msgs/msg/world_entity.hpp>
#include <thorp_msgs/srv/query_world.hpp>

namespace thorp::bt::actions
{
using thorp_msgs::msg::ThorpError;

/**
 * The table the robot is at, as thorp_world tells: the one it docked at, or else the closest. Fails with NOT_AT_TABLE
 * if it's at none.
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
    ports.insert({ BT::OutputPort<std::string>("table_id"),  //
                   BT::OutputPort<int>("error") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (response.at_table.empty())
    {
      RCLCPP_WARN(logger(*this), "Not at any table%s%s", response.message.empty() ? "" : ": ",
                  response.message.c_str());
      setOutput("error", static_cast<int>(ThorpError::NOT_AT_TABLE));
      return BT::NodeStatus::FAILURE;
    }
    setOutput("table_id", response.at_table);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailedRequest(FailureCause failure) override
  {
    setOutput("error", static_cast<int>(ThorpError::SERVER_NOT_AVAILABLE));
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(GetCurrentTable);
};
}  // namespace thorp::bt::actions
