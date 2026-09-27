#pragma once

#include <string>

#include <behaviortree_cpp/behavior_tree.h>

// Nav2's string conversions, as for geometry_msgs::msg::PoseStamped ('stamp;frame;x;y;z;qx;qy;qz;qw' or 'json:{...}'),
// used by Nav2's BT nodes; every translation unit must see the same conversions, so all Thorp nodes include them
#include <nav2_behavior_tree/bt_utils.hpp>

#include <thorp_toolkit/geometry.hpp>

namespace thorp::bt
{
/**
 * Parse a pose in Thorp's short forms, 'x;y;yaw;frame' or 'x;y;z;roll;pitch;yaw;frame', as bt_server callers write
 * them.
 * @throw BT::RuntimeError for other forms
 */
inline geometry_msgs::msg::PoseStamped parsePose(BT::StringView str)
{
  auto parts = BT::splitString(str, ';');
  if (parts.size() == 4)
  {
    return toolkit::createPose(BT::convertFromString<double>(parts[0]), BT::convertFromString<double>(parts[1]),
                               BT::convertFromString<double>(parts[2]), std::string(parts[3]));
  }
  if (parts.size() == 7)
  {
    return toolkit::createPose(BT::convertFromString<double>(parts[0]), BT::convertFromString<double>(parts[1]),
                               BT::convertFromString<double>(parts[2]), BT::convertFromString<double>(parts[3]),
                               BT::convertFromString<double>(parts[4]), BT::convertFromString<double>(parts[5]),
                               std::string(parts[6]));
  }
  throw BT::RuntimeError("Invalid number of inputs to build a PoseStamped: ", std::string(str));
}

}  // namespace thorp::bt
