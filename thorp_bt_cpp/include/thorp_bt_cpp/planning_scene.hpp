#pragma once

#include <sstream>
#include <string>
#include <vector>

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

}  // namespace thorp::bt
