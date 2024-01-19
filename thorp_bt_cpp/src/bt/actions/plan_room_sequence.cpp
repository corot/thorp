#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_action_node.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <ipa_building_msgs/FindRoomSequenceWithCheckpointsAction.h>

namespace thorp::bt::actions
{
class PlanRoomSequence : public BT::RosActionNode<ipa_building_msgs::FindRoomSequenceWithCheckpointsAction>
{
public:
  PlanRoomSequence(const std::string& name, const BT::NodeConfiguration& config) : RosActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosActionNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("exploration/room_sequence_planning");
    ports.insert({ BT::InputPort<geometry_msgs::PoseStamped>("robot_pose"),  //
                   BT::InputPort<float>("robot_radius"),                     //
                   BT::InputPort<sensor_msgs::Image>("map_image"),           //
                   BT::InputPort<geometry_msgs::Pose>("map_origin"),         //
                   BT::InputPort<float>("map_resolution"),                   //
                   BT::InputPort<std::vector<ipa_building_msgs::RoomInformation>>("room_information_in_pixel"),
                   BT::OutputPort<std::vector<uint32_t>>("room_sequence") });
    return ports;
  }

private:
  std::optional<GoalType> getGoal() override
  {
    if (status() == BT::NodeStatus::RUNNING)
      return std::nullopt;

    GoalType goal;
    goal.robot_start_coordinate = getInput<geometry_msgs::PoseStamped>("robot_pose")->pose;
    goal.robot_radius = *getInput<float>("robot_radius");
    goal.input_map = *getInput<sensor_msgs::Image>("map_image");
    goal.map_origin = *getInput<geometry_msgs::Pose>("map_origin");
    goal.map_resolution = *getInput<float>("map_resolution");
    goal.room_information_in_pixel =
        *getInput<std::vector<ipa_building_msgs::RoomInformation>>("room_information_in_pixel");

    return goal;
  }

  BT::NodeStatus onSucceeded(const ResultConstPtr& res) override
  {
    auto room_sequence = res->checkpoints.front().room_indices;
    // room numbers start with 1, so we get them with index + 1
    std::for_each(room_sequence.begin(), room_sequence.end(), [](uint32_t& index) { ++index; });
    setOutput("room_sequence", room_sequence);

    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(PlanRoomSequence);
};
}  // namespace thorp::bt::actions
