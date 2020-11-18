#include <json/json.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;

#include "thorp_costmap_layers/scene_interface.h"


namespace thorp_costmap_layers
{

SceneInterface::SceneInterface(ros::NodeHandle& nh, tf2_ros::Buffer& tf, const std::string& map_frame,
                               std::function<void(double, double, double)> update_map_callback) :
  BaseInterface(nh, tf, map_frame),
  callback_processed_(false),
  update_map_callback_(update_map_callback)
{
  // Provide a topic to add semantic objects to the costmap
  add_objs_sub_ = nh_.subscribe("add_objects", 10, &SceneInterface::sceneMessageCallback, this);

  // List current objects for inspection in a latched topic  TODO not true now,,,  not really used
  objects_pub_ = nh_.advertise<moveit_msgs::PlanningSceneWorld>("objects", 10, true);
}

SceneInterface::~SceneInterface()
{
}

void SceneInterface::sceneMessageCallback(const moveit_msgs::PlanningSceneWorldConstPtr& planning_scene)
{
  if (planning_scene->collision_objects.empty())
    return;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  processCollisionObjs(planning_scene->collision_objects);
}

void SceneInterface::processCollisionObjs(const std::vector<moveit_msgs::CollisionObject>& collision_objects)
{
  try
  {
    Json::Reader reader;
    Json::Value db_info;

    bool trigger_map_update = false;
    for (const auto& co : collision_objects)
    {
      // Discard objects of unknown type
      reader.parse(co.type.db, db_info);
      std::string type = db_info["type"].asString();
      if (object_types_.find(type) == object_types_.end())
      {
        ROS_WARN("[scene_interface] Discarded object %s of unknown type %s", co.id.c_str(), type.c_str());
        continue;
      }

      if (co.operation == moveit_msgs::CollisionObject::ADD    ||
          co.operation == moveit_msgs::CollisionObject::APPEND || // TODO what's the difference?
          co.operation == moveit_msgs::CollisionObject::MOVE)
      {
        // transform obstacle to correct frame
        std::vector<std::vector<geometry_msgs::PoseStamped>> contours;
        collisionObjToContours(co, contours, object_types_[type].length_padding, object_types_[type].width_padding);
        // convert into obstacles and save into hash -> also inserts them into updated
        contoursToHash(contours, co.id, object_types_[type]);

        if (object_types_[type].force_update)
          trigger_map_update = true;

        ROS_DEBUG("[scene_interface] Add object %s of type %s", co.id.c_str(), type.c_str());
      }
      else if (co.operation == moveit_msgs::CollisionObject::REMOVE)
      {
        removeContours(co.id);
        ROS_DEBUG("[scene_interface] Removed object %s of type %s", co.id.c_str(), type.c_str());
      }
    }

    if (trigger_map_update)
    {
      // if any of the added objects must appear in the costmap asap, force updating without waiting for update thread
      geometry_msgs::TransformStamped transform = tf_.lookupTransform(map_frame_, robot_frame_, ros::Time(0.0));
      update_map_callback_(transform.transform.translation.x, transform.transform.translation.x, ttk::yaw(transform));
      callback_processed_ = true;
    }
  }
  catch (Json::LogicError &e)
  {
    ROS_ERROR("[scene_interface] Json LogicError: %s", e.what());
  }
}

// transform primitive obstacle to current frame of db
void SceneInterface::collisionObjToContours(const moveit_msgs::CollisionObject& collision_object,
                                            std::vector<std::vector<geometry_msgs::PoseStamped>>& contours,
                                            double length_padding, double width_padding) const
{
  contours.resize(collision_object.primitives.size());

  // we should be able to transform obstacles instantly
  if (!tf_.canTransform(map_frame_, collision_object.header.frame_id, ros::Time(0), ros::Duration(1.0)))
  {
    ROS_WARN("[scene_interface] Transform collision object frame '%s' to map frame '%s' not available after 1 second",
             collision_object.header.frame_id.c_str(), map_frame_.c_str());
    return;
  }

  for (int it_primitive = 0; it_primitive < collision_object.primitives.size(); ++it_primitive)
  {
    contours[it_primitive].resize(4);

    // safety check to make sure we process only box obstacles; TODO: support other primitives
    if (collision_object.primitives[it_primitive].type != shape_msgs::SolidPrimitive::BOX)
    {
      ROS_WARN("[scene_interface] One of the obstacles received on the topic was not a box, skipping drawing it in map");
      continue;
    }

    // transform the box to the map frame
    double length = collision_object.primitives[it_primitive].dimensions[0] + length_padding * 2;
    double width  = collision_object.primitives[it_primitive].dimensions[1] + width_padding * 2;
    double height = collision_object.primitives[it_primitive].dimensions[2];

    // generate poses with corners of the box
    std::vector<geometry_msgs::PoseStamped> corner_poses(4);

    // Consider primitive rotation (just yaw)
    double yaw = ttk::yaw(collision_object.primitive_poses[it_primitive]);
    double xcos, ycos, xsin, ysin, sin_yaw = sin(yaw), cos_yaw = cos(yaw);
    size_t i = 0;
    for (double y = -0.5; y <= 0.5; ++y)
    {
      for (double x = -0.5; x <= 0.5; ++x)
      {
        xcos = x * length * cos_yaw, xsin = x * length * sin_yaw;
        ycos = y * width  * cos_yaw, ysin = y * width  * sin_yaw;
        corner_poses[i].pose = collision_object.primitive_poses[it_primitive];
        corner_poses[i].pose.position.x = corner_poses[i].pose.position.x - (xcos - ysin);
        corner_poses[i].pose.position.y = corner_poses[i].pose.position.y - (xsin + ycos);
        corner_poses[i].pose.position.z = corner_poses[i].pose.position.z - 0.5 * height;
        corner_poses[i].header.frame_id = collision_object.header.frame_id;
        corner_poses[i].header.stamp = ros::Time(0);

        tf_.transform(corner_poses[i], contours[it_primitive][i], map_frame_);

        ++i;
      }
    }
    // correctly order the box so it can be drawn
    std::swap(contours[it_primitive][1], contours[it_primitive][2]);
    std::swap(contours[it_primitive][2], contours[it_primitive][3]);
  }
}

void SceneInterface::publishObject(const Object& object)
{
  moveit_msgs::PlanningSceneWorld new_message;
  new_message.collision_objects[0].operation = moveit_msgs::CollisionObject::ADD;
  new_message.collision_objects[0].id = std::to_string(object.id);
  geometry_msgs::Point temp_point;
  temp_point.z = 0.0f;
  int i = 0;
  for (const auto& pt : object.contour_points)
  {
    temp_point.x = pt.x;
    temp_point.y = pt.y;
    new_message.collision_objects[0].meshes[0].vertices[i] = temp_point;
    ++i;
  }
  objects_pub_.publish(new_message);
}

// delete object based on id
void SceneInterface::deleteObject(const Object& object)
{
  moveit_msgs::PlanningSceneWorld delete_message;
  delete_message.collision_objects[0].operation = 1;
  delete_message.collision_objects[0].id = std::to_string(object.id);
  objects_pub_.publish(delete_message);
}

}
