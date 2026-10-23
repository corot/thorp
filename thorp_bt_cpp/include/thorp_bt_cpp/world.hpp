#pragma once

#include <moveit_msgs/msg/collision_object.hpp>
#include <nlohmann/json.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <thorp_msgs/msg/world_entity.hpp>

namespace thorp::bt
{
/**
 * A table thorp_world knows, as the table nodes take it: a box of its size, with its pose, on the map frame.
 */
inline moveit_msgs::msg::CollisionObject tableObject(const thorp_msgs::msg::WorldEntity& entity)
{
  moveit_msgs::msg::CollisionObject table;
  table.id = entity.id;
  table.header = entity.pose.header;
  table.pose = entity.pose.pose;
  table.operation = moveit_msgs::msg::CollisionObject::ADD;
  table.primitives.resize(1);
  table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  table.primitives[0].dimensions = { entity.size.x, entity.size.y, entity.size.z };
  table.primitive_poses.resize(1);
  table.primitive_poses[0].orientation.w = 1.0;
  // as detections describe tables
  table.type.db =
      nlohmann::json{ { "size", { entity.size.x, entity.size.y, entity.size.z } }, { "color", entity.color } }.dump();
  return table;
}

}  // namespace thorp::bt
