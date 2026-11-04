#pragma once

#include <string>

#include <behaviortree_cpp/behavior_tree.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

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

namespace BT
{
/**
 * Poses written in trees, in Thorp's short forms; every translation unit must see the same conversions, so all Thorp
 * nodes include this
 */
template <>
inline geometry_msgs::msg::PoseStamped convertFromString(StringView str)
{
  return thorp::bt::parsePose(str);
}

}  // namespace BT
