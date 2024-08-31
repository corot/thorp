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

std::string getIDs(const std::vector<moveit_msgs::CollisionObject>& objects)
{
  if (objects.empty())
  {
    return "";
  }

  std::ostringstream oss;
  std::transform(objects.begin(), objects.end() - 1, std::ostream_iterator<std::string>(oss, ", "),
                 [](const auto& obj) { return obj.id; });
  oss << objects.back().id;

  return oss.str();
}

PlanningScene& PlanningScene::instance()
{
  static PlanningScene instance;
  return instance;
}

moveit_msgs::CollisionObject PlanningScene::getObject(const std::string& obj_name)
{
  // Look for obj_name in the list of collision objects
  std::map<std::string, moveit_msgs::CollisionObject> objects = getObjects(std::vector<std::string>{ obj_name });

  if (objects.empty())
  {
    ROS_ERROR("[planning scene] Collision object '%s' not found", obj_name.c_str());
    throw std::invalid_argument("Collision object '" + obj_name + "' not found");
  }

  if (objects.size() > 1)
  {
    // This should not happen, as object detection tries to provide unique names to all objects...
    ROS_WARN("[planning scene] More than one (%lu) collision objects with id '%s' found!", objects.size(),
             obj_name.c_str());
  }

  return objects[obj_name];
}

void PlanningScene::addObject(const std::string& name, const geometry_msgs::PoseStamped& pose,
                              const std::vector<double>& size, const std::string& color)
{
  moveit_msgs::CollisionObject co;
  co.id = name;
  co.header = pose.header;
  co.pose = pose.pose;
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions = size;
  moveit_msgs::ObjectColor obj_color;
  obj_color.id = co.id;
  obj_color.color = namedColor(color);
  addCollisionObjects({ co }, { obj_color });
}

void PlanningScene::removeAll(const std::set<std::string>& exempted)
{
  auto objs_to_remove = getKnownObjectNames();
  for (const auto& obj : exempted)
  {
    auto it = std::find(objs_to_remove.begin(), objs_to_remove.end(), obj);
    if (it != objs_to_remove.end())
    {
      objs_to_remove.erase(it);
    }
  }
  removeCollisionObjects(objs_to_remove);
}

void PlanningScene::removeObject(const std::string& obj_name)
{
  std::vector<std::string> objs_to_remove{ obj_name };
  removeCollisionObjects(objs_to_remove);
}

void PlanningScene::displaceObject(const std::string& obj_name, const geometry_msgs::PoseStamped& new_pose)
{
  auto co = getObject(obj_name);
  co.header = new_pose.header;
  co.pose = new_pose.pose;
  co.operation = moveit_msgs::CollisionObject::MOVE;
  applyCollisionObject(co);
}

int32_t PlanningScene::extractObjectData(const moveit_msgs::CollisionObject& obj, geometry_msgs::PoseStamped& obj_pose,
                                         geometry_msgs::Vector3& obj_size)
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

int32_t PlanningScene::getObjectData(const std::string& obj_name, geometry_msgs::PoseStamped& obj_pose,
                                     geometry_msgs::Vector3& obj_size)
{
  // Look for obj_name in the list of collision objects
  std::map<std::string, moveit_msgs::CollisionObject> objects = getObjects(std::vector<std::string>(1, obj_name));

  if (objects.empty())
  {
    ROS_ERROR("[planning scene] Collision object '%s' not found", obj_name.c_str());
    return thorp_msgs::ThorpError::OBJECT_NOT_FOUND;
  }

  if (objects.size() > 1)
  {
    // This should not happen, as object detection tries to provide unique names to all objects...
    ROS_WARN("[planning scene] More than one (%lu) collision objects with name '%s' found!", objects.size(),
             obj_name.c_str());
  }

  return extractObjectData(objects[obj_name], obj_pose, obj_size);
}

int32_t PlanningScene::getAttachedObjectData(const std::string& obj_name, geometry_msgs::PoseStamped& obj_pose,
                                             geometry_msgs::Vector3& obj_size)
{
  // Look for obj_name in the list of attached objects
  std::map<std::string, moveit_msgs::AttachedCollisionObject> objects =
      getAttachedObjects(std::vector<std::string>(1, obj_name));

  if (objects.empty())
  {
    // Maybe pick failed; we will not continue because place will surely fail without knowing the attaching pose
    ROS_ERROR("[planning scene] Attached collision object '%s' not found", obj_name.c_str());
    return thorp_msgs::ThorpError::OBJECT_NOT_FOUND;
  }

  if (objects.size() > 1)
  {
    // This should not happen... we grasped two objects with the same name???
    ROS_WARN("[planning scene] More than one (%lu) attached collision objects with name '%s' found!", objects.size(),
             obj_name.c_str());
  }

  return extractObjectData(objects[obj_name].object, obj_pose, obj_size);
}

}  // namespace thorp::toolkit
