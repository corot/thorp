#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"
#include "thorp_bt_cpp/bt/actions/action_client_node.hpp"

#include <nav_msgs/OccupancyGrid.h>
#include <ipa_building_msgs/FindRoomSequenceWithCheckpointsAction.h>

#include <thorp_toolkit/common.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class PlanRoomSequence : public BT::SimpleActionClientNode<ipa_building_msgs::FindRoomSequenceWithCheckpointsAction>
{
public:
  PlanRoomSequence(const std::string& name, const BT::NodeConfiguration& config) : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::PoseStamped>("robot_pose"),
             BT::InputPort<float>("robot_radius"),
             BT::InputPort<sensor_msgs::Image>("map_image"),
             BT::InputPort<geometry_msgs::Pose>("map_origin"),
             BT::InputPort<float>("map_resolution"),
             BT::InputPort<std::vector<ipa_building_msgs::RoomInformation>>("room_information_in_pixel"),
             BT::OutputPort<std::vector<uint32_t>>("room_sequence") };
  }

  bool setGoal(GoalType& goal) override
  {
    goal.robot_start_coordinate = getInput<geometry_msgs::PoseStamped>("robot_pose")->pose;
    goal.robot_radius = *getInput<float>("robot_radius");
    goal.input_map = *getInput<sensor_msgs::Image>("map_image");
    goal.map_origin = *getInput<geometry_msgs::Pose>("map_origin");
    goal.map_resolution = *getInput<float>("map_resolution");
    goal.room_information_in_pixel =
        *getInput<std::vector<ipa_building_msgs::RoomInformation>>("room_information_in_pixel");

    return true;
  }

  BT::NodeStatus onSuccess(const ResultConstPtr& res)
  {
    auto room_sequence = res->checkpoints.front().room_indices;
    // room numbers start with 1, so we get them with index + 1
    std::for_each(room_sequence.begin(), room_sequence.end(), [](uint32_t& index) { ++index; });
    setOutput("room_sequence", room_sequence);

    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace thorp::bt::actions
