#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/errors.hpp"
#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_msgs/msg/thorp_error.hpp>
#include <thorp_msgs/msg/world_entity.hpp>
#include <thorp_msgs/srv/query_world.hpp>

namespace thorp::bt::conditions
{
using thorp_msgs::msg::ThorpError;
using thorp_msgs::msg::WorldEntity;

/**
 * Whether thorp_world says an object can be picked from where the robot is: on a table, and within the arm's reach.
 * Otherwise fails with the reason as error: OBJECT_NOT_FOUND, OBJECT_NOT_ON_TABLE (carried, or missing from where it
 * was seen), NOT_AT_TABLE (out of reach, on a table the robot isn't at) or OUT_OF_REACH (on the robot's table).
 */
class ObjectPickable : public BT::RosServiceNode<thorp_msgs::srv::QueryWorld, BT::ConditionNode>
{
public:
  ObjectPickable(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("world/query");
    ports.insert({ BT::InputPort<std::string>("object_id"),                        //
                   BT::OutputPort<std::string>("table_id", "the object's table"),  //
                   BT::OutputPort<ThorpError>("error") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.ids = { requireInput<std::string>(*this, "object_id") };
    request.kind = WorldEntity::OBJECT;
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    const std::string object_id = requireInput<std::string>(*this, "object_id");
    if (response.entities.empty())
      return fail(ThorpError::OBJECT_NOT_FOUND, "no object '" + object_id + "'");
    const WorldEntity& object = response.entities.front();
    if (!object.has_pose || object.status == WorldEntity::MISSING)
      return fail(ThorpError::OBJECT_NOT_ON_TABLE,
                  "'" + object_id + "' is " + (object.has_pose ? "missing" : "on the " + object.support));
    setOutput("table_id", object.support);
    if (response.reachable.front())
      return BT::NodeStatus::SUCCESS;
    if (response.at_table != object.support)
    {
      const std::string robot_at = response.at_table.empty() ? "at no table" : "at '" + response.at_table + "'";
      return fail(ThorpError::NOT_AT_TABLE, "'" + object_id + "' is on '" + object.support + "', robot " + robot_at);
    }
    return fail(ThorpError::OUT_OF_REACH, "'" + object_id + "' is beyond the arm's reach");
  }

  BT::NodeStatus onFailedRequest(FailureCause failure) override
  {
    setOutput("error", makeError(ThorpError::SERVER_NOT_AVAILABLE,
                                 "no answer from thorp_world's " +
                                     getInput<std::string>("service_name").value_or("query service")));
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus fail(int32_t error, const std::string& reason)
  {
    RCLCPP_WARN(logger(*this), "Can't pick: %s", reason.c_str());
    setOutput("error", makeError(error, reason));
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(ObjectPickable);
};
}  // namespace thorp::bt::conditions
