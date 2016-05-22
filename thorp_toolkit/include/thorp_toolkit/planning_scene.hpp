/*
 * planning_scene.hpp
 *
 *  Created on: May 22, 2016
 *      Author: jorge
 *
 *  Use MoveIt's planning scene interface to gather information of collision/attached objects
 */

#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>


namespace thorp_toolkit
{

/**
 * We use the planning scene to gather information of collision/attached objects
 * Note that we use the Construct On First Use idiom to allow using static attribute for planning scene
 * interface while avoiding the static initialization disaster (the planning scene cannot be initialized
 * until ROS is up and running).
 */
moveit::planning_interface::PlanningSceneInterface& planningScene();

/**
 * Look for obj_name in the list of collision objects and retrieve its pose and size
 * @param obj_name Object to look for
 * @param obj_pose Resulting object pose
 * @param obj_size Resulting object size
 * @return A ThorpError code, 1 for SUCCESS, negative values for failures
 */
int32_t getObjectData(const std::string& obj_name,
                      geometry_msgs::PoseStamped& obj_pose, geometry_msgs::Vector3& obj_size);

/**
 * Look for obj_name in the list of attached collision objects and retrieve its pose relative to the attaching frame
 * @param obj_name Attached object to look for
 * @param obj_pose Resulting object pose. No stamped, as it's relative to attaching frame (gripper_link)
 * @return A ThorpError code, 1 for SUCCESS, negative values for failures
 */
int32_t getAttachedObjectPose(const std::string& obj_name, geometry_msgs::Pose& obj_pose);

}
