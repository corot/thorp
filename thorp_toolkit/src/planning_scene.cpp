/*
 * planning_scene.cpp
 *
 *  Created on: May 22, 2016
 *      Author: jorge
 */

#include <tf/tf.h>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

#include <thorp_msgs/ThorpError.h>

#include "thorp_toolkit/planning_scene.hpp"


namespace thorp::toolkit
{

moveit::planning_interface::PlanningSceneInterface& planningScene()
{
  static moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  return planning_scene_interface;
}

int32_t extractObjectData(const moveit_msgs::CollisionObject& obj,
                          geometry_msgs::PoseStamped& obj_pose, geometry_msgs::Vector3& obj_size)
{
  obj_pose.header = obj.header;
  obj_pose.pose = obj.pose;

  // We get object's size from either the first mesh or first primitive (we assume there aren't composed objects)
  if (!obj.meshes.empty())
  {
    tf::vectorEigenToMsg(shapes::computeShapeExtents(obj.meshes[0]), obj_size);
  }
  else if (!obj.primitives.empty())
  {
    tf::vectorEigenToMsg(shapes::computeShapeExtents(obj.primitives[0]), obj_size);
  }
  else
  {
    ROS_ERROR("[planning scene] Collision object '%s' has neither meshes or primitives", obj.id.c_str());
    return thorp_msgs::ThorpError::OBJECT_POSE_NOT_FOUND;
  }

  return thorp_msgs::ThorpError::SUCCESS;
}

int32_t getObjectData(const std::string& obj_name,
                      geometry_msgs::PoseStamped& obj_pose, geometry_msgs::Vector3& obj_size)
{
  // Look for obj_name in the list of collision objects
  std::map<std::string, moveit_msgs::CollisionObject> objects =
      planningScene().getObjects(std::vector<std::string>(1, obj_name));

  if (objects.empty())
  {
    ROS_ERROR("[planning scene] Collision object '%s' not found", obj_name.c_str());
    return thorp_msgs::ThorpError::OBJECT_NOT_FOUND;
  }

  if (objects.size() > 1)
  {
    // This should not happen, as object detection tries to provide unique names to all objects...
    ROS_WARN("[planning scene] More than one (%lu) collision objects with name '%s' found!",
             objects.size(), obj_name.c_str());
  }

  return extractObjectData(objects[obj_name], obj_pose, obj_size);
}


int32_t getAttachedObjectData(const std::string& obj_name,
                              geometry_msgs::PoseStamped& obj_pose, geometry_msgs::Vector3& obj_size)
{
  // Look for obj_name in the list of attached objects
  std::map<std::string, moveit_msgs::AttachedCollisionObject> objects =
      planningScene().getAttachedObjects(std::vector<std::string>(1, obj_name));

  if (objects.empty())
  {
    // Maybe pick failed; we will not continue because place will surely fail without knowing the attaching pose
    ROS_ERROR("[planning scene] Attached collision object '%s' not found", obj_name.c_str());
    return thorp_msgs::ThorpError::OBJECT_NOT_FOUND;
  }

  if (objects.size() > 1)
  {
    // This should not happen... we grasped two objects with the same name???
    ROS_WARN("[planning scene] More than one (%lu) attached collision objects with name '%s' found!",
             objects.size(), obj_name.c_str());
  }

  return extractObjectData(objects[obj_name].object, obj_pose, obj_size);
}

}
