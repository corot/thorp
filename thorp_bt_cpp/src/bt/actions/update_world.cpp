#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <map>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thorp_msgs/srv/update_world.hpp>

namespace thorp::bt::actions
{
/**
 * Report a skill's effect to thorp_world: an object picked, placed on a table at a pose, stowed on a tray slot or
 * disturbed; the robot docked at a table or undocked. Fails if the world refuses it, as for an unknown id.
 */
class UpdateWorld : public BT::RosServiceNode<thorp_msgs::srv::UpdateWorld, BT::SyncActionNode>
{
public:
  UpdateWorld(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("world/update");
    ports.insert({ BT::InputPort<std::string>("effect", "picked, placed, stowed, disturbed, docked or undocked"),
                   BT::InputPort<std::string>("object_id", "picked, placed, stowed or disturbed"),
                   BT::InputPort<std::string>("table_id", "placed on or docked at"),
                   BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "where placed"),
                   BT::InputPort<int>("slot", -1, "tray slot stowed on") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    static const std::map<std::string, uint8_t> effects = {
      { "picked", RequestType::PICKED },       { "placed", RequestType::PLACED }, { "stowed", RequestType::STOWED },
      { "disturbed", RequestType::DISTURBED }, { "docked", RequestType::DOCKED }, { "undocked", RequestType::UNDOCKED }
    };
    const std::string effect = requireInput<std::string>(*this, "effect");
    const auto it = effects.find(effect);
    if (it == effects.end())
      throw BT::RuntimeError(name(), ": unknown effect '", effect, "'");
    request.effect = it->second;
    request.object_id = getInput<std::string>("object_id").value_or("");
    request.table_id = getInput<std::string>("table_id").value_or("");
    request.slot = getInput<int>("slot").value_or(-1);
    if (auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose"))
      request.pose = *pose;
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (!response.success)
    {
      RCLCPP_ERROR(logger(*this), "The world refused the update: %s", response.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(UpdateWorld);
};
}  // namespace thorp::bt::actions
