#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"
#include "thorp_bt_cpp/bt/actions/action_client_node.hpp"

#include <nav_msgs/OccupancyGrid.h>
#include <ipa_building_msgs/MapSegmentationAction.h>

#include <thorp_toolkit/common.hpp>
namespace ttk = thorp_toolkit;

namespace thorp::bt::actions
{
class SegmentRooms : public BT::SimpleActionClientNode<ipa_building_msgs::MapSegmentationAction>
{
public:
  SegmentRooms(const std::string& name, const BT::NodeConfiguration& config) : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<sensor_msgs::Image>("map_image"),
             BT::OutputPort<geometry_msgs::Pose>("map_origin"),
             BT::OutputPort<float>("map_resolution"),
             BT::OutputPort<float>("robot_radius"),
             BT::OutputPort<sensor_msgs::Image>("segmented_map"),
             BT::OutputPort<std::vector<ipa_building_msgs::RoomInformation>>("room_information_in_meter"),
             BT::OutputPort<std::vector<ipa_building_msgs::RoomInformation>>("room_information_in_pixel") };
  }

  bool createGoal(GoalType& goal)
  {
    auto map = ttk::waitForMessage<nav_msgs::OccupancyGrid>("map");
    if (!map)
    {
      ROS_ERROR_NAMED(name(), "Unable to retrieve map");
      return false;
    }
    goal.input_map.header = map->header;
    goal.input_map.height = map->info.height;
    goal.input_map.width = map->info.width;
    goal.input_map.step = map->info.width;
    goal.input_map.encoding = "mono8";
    goal.input_map.data.resize(map->info.height * map->info.width, 255);
    // Convert map into a black and white 8bit single-channel image (format 8UC1), which is 0 (black)
    // for obstacles and unknown space, and 255 (white) for free space
    for (int i = 0; i < map->info.height; ++i)
    {
      for (int j = 0; j < map->info.width; ++j)
      {
        if (map->data[i * map->info.width + j] == -1 || map->data[i * map->info.width + j] == 100)
          goal.input_map.data[i * map->info.width + j] = 0;
      }
    }
    goal.map_origin = map->info.origin;
    goal.map_resolution = map->info.resolution;
    goal.return_format_in_meter = true;
    goal.return_format_in_pixel = true;
    goal.robot_radius = ros::NodeHandle().param<double>("move_base_flex/global_costmap/robot_radius", 0.18);

    // those values are also needed by PlanRoomSequence and PlanRoomExploration, so share them on output ports
    // the segmented map comes with its own origin and resolution, but both are the same as for the input map
    setOutput("map_image", goal.input_map);
    setOutput("map_origin", goal.map_origin);
    setOutput("map_resolution", goal.map_resolution);
    setOutput("robot_radius", goal.robot_radius);

    return true;
  }

  BT::NodeStatus onSuccess(const ResultConstPtr& res)
  {
    setOutput("segmented_map", res->segmented_map);
    setOutput("room_information_in_meter", res->room_information_in_meter);
    setOutput("room_information_in_pixel", res->room_information_in_pixel);

    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace thorp::bt::actions
