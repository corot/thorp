#pragma once

#include <sstream>
#include <string>
#include <vector>

#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

namespace thorp::bt
{
/**
 * MoveIt's planning scene interface, shared by all the nodes that query or modify the planning scene.
 * Its calls are synchronous and it spins its own node, so nodes can use it within a tick.
 */
inline moveit::planning_interface::PlanningSceneInterface& planningScene()
{
  static moveit::planning_interface::PlanningSceneInterface planning_scene("", false);
  return planning_scene;
}

/**
 * Objects' names, comma-separated, for logging.
 */
inline std::string objectIds(const std::vector<moveit_msgs::msg::CollisionObject>& objects)
{
  std::ostringstream ids;
  for (size_t i = 0; i < objects.size(); ++i)
    ids << (i ? ", " : "") << objects[i].id;
  return ids.str();
}

/**
 * Height of an object made of a single shape, as perception makes them; zero if it has none.
 */
inline double objectHeight(const moveit_msgs::msg::CollisionObject& object)
{
  if (!object.meshes.empty())
    return shapes::computeShapeExtents(shapes::ShapeMsg(object.meshes.front())).z();
  if (!object.primitives.empty())
    return shapes::computeShapeExtents(shapes::ShapeMsg(object.primitives.front())).z();
  return 0.0;
}

}  // namespace thorp::bt
