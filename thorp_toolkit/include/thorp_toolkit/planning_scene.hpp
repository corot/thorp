/*
 * planning_scene.hpp
 *
 *  Created on: May 22, 2016
 *      Author: jorge
 *
 *  Use MoveIt's planning scene interface to gather information of collision/attached objects
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "thorp_toolkit/common.hpp"

namespace thorp::toolkit
{
std::string getIDs(const std::vector<moveit_msgs::CollisionObject>& objects);

class PlanningScene : public moveit::planning_interface::PlanningSceneInterface
{
public:
  static PlanningScene& instance();

  moveit_msgs::CollisionObject getObject(const std::string& obj_name);

  void addObject(const std::string& name, const geometry_msgs::PoseStamped& pose, const std::vector<double>& size,
                 const std::string& color);

  void removeAll(const std::set<std::string>& exempted = {});

  void removeObject(const std::string& obj_name);

  void displaceObject(const std::string& obj_name, const geometry_msgs::PoseStamped& new_pose);

  /**
   * Extract pose and size from a collision object
   * @param obj Source collision object
   * @param obj_pose Extracted object pose
   * @param obj_size Extracted object size
   * @return A ThorpError code, 1 for SUCCESS, negative values for failures
   */
  int32_t extractObjectData(const moveit_msgs::CollisionObject& obj, geometry_msgs::PoseStamped& obj_pose,
                            geometry_msgs::Vector3& obj_size);

  /**
   * Look for obj_name in the list of collision objects and retrieve its pose and size
   * @param obj_name Object to look for
   * @param obj_pose Resulting object pose
   * @param obj_size Resulting object size
   * @return A ThorpError code, 1 for SUCCESS, negative values for failures
   */
  int32_t getObjectData(const std::string& obj_name, geometry_msgs::PoseStamped& obj_pose,
                        geometry_msgs::Vector3& obj_size);

  /**
   * Look for obj_name in the list of attached collision objects and retrieve its pose and size
   * @param obj_name Attached object to look for
   * @param obj_pose Resulting object pose, relative to attaching frame (normally gripper_link)
   * @return A ThorpError code, 1 for SUCCESS, negative values for failures
   */
  int32_t getAttachedObjectData(const std::string& obj_name, geometry_msgs::PoseStamped& obj_pose,
                                geometry_msgs::Vector3& obj_size);

private:
  PlanningScene() = default;

  // Disable copy and assignment
  PlanningScene(const PlanningScene&) = delete;
  PlanningScene& operator=(const PlanningScene&) = delete;
};

}  // namespace thorp::toolkit
