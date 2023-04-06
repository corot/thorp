#include "thorp_costmap_layers/srv_interface.hpp"

#include <json/json.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace thorp_costmap_layers
{

ServiceInterface::ServiceInterface(ros::NodeHandle& nh, tf2_ros::Buffer& tf, costmap_2d::LayeredCostmap* lcm)
  : BaseInterface(nh, tf, lcm)
{
  // Service to add/modify/remove semantic objects to the costmap
  update_srv_ = nh_.advertiseService("update_objects", &ServiceInterface::updateCollisionObjs, this);
}

ServiceInterface::~ServiceInterface() = default;

void ServiceInterface::reconfigure(const thorp_costmap_layers::SemanticLayerConfig& config)
{
  enabled_ = config.service_enabled;
}

bool ServiceInterface::updateCollisionObjs(thorp_msgs::UpdateCollisionObjs::Request& request,
                                           thorp_msgs::UpdateCollisionObjs::Response& response)
{
  try
  {
    Json::Reader reader;
    Json::Value db_info;

    response.error.code = thorp_msgs::ThorpError::SUCCESS;
    response.error.text = "Object successfully updated";
    bool trigger_map_update = false;
    for (const auto& co : request.collision_objects)
    {
      // Discard objects of unknown type
      reader.parse(co.type.db, db_info);
      std::string type = db_info["type"].asString();
      if (object_types_.find(type) == object_types_.end())
      {
        ROS_WARN_NAMED(LOGNAME, "Discarded object %s of unknown type %s", co.id.c_str(), type.c_str());
        response.error.code = thorp_msgs::ThorpError::INVALID_OBJECT_TYPE;
        response.error.text = type;
        break;
      }
      if (co.operation == moveit_msgs::CollisionObject::ADD || co.operation == moveit_msgs::CollisionObject::APPEND ||
          co.operation == moveit_msgs::CollisionObject::MOVE)
      {
        // transform obstacle to correct frame
        std::vector<std::vector<geometry_msgs::PoseStamped>> contours;
        collisionObjToContours(co, contours, object_types_[type].length_padding, object_types_[type].width_padding);
        // convert into obstacles and save into hash -> also inserts them into updated
        contoursToHash(contours, co.id, type);

        ROS_DEBUG_NAMED(LOGNAME, "Add object %s of type %s", co.id.c_str(), type.c_str());
      }
      else if (co.operation == moveit_msgs::CollisionObject::REMOVE)
      {
        if (removeContours(co.id))
        {
          ROS_DEBUG_NAMED(LOGNAME, "Removed object %s of type %s", co.id.c_str(), type.c_str());
        }
        else
        {
          ROS_WARN_NAMED(LOGNAME, "Attempting to remove unknown object %s", co.id.c_str());
          response.error.code = thorp_msgs::ThorpError::OBJECT_NOT_FOUND;
          response.error.text = co.id;
          break;
        }
      }

      if (object_types_[type].force_update)
        trigger_map_update = true;
    }
    if (trigger_map_update)
    {
      // one or more added/removed objects require immediate update: force update instead of waiting for update thread
      layered_costmap_->updateMap(robot_x_, robot_y_, robot_yaw_);
    }
  }
  catch (Json::LogicError& e)
  {
    ROS_ERROR_NAMED(LOGNAME, "Json LogicError: %s", e.what());
    response.error.code = thorp_msgs::ThorpError::INVALID_OBJECT_TYPE;
    response.error.text = e.what();
  }
  return response.error.code == thorp_msgs::ThorpError::SUCCESS;
}

// transform primitive obstacle to current frame of db
void ServiceInterface::collisionObjToContours(const moveit_msgs::CollisionObject& collision_object,
                                              std::vector<std::vector<geometry_msgs::PoseStamped>>& contours,
                                              double length_padding, double width_padding) const
{
  contours.resize(collision_object.primitives.size());

  // we should be able to transform obstacles instantly
  if (!tf_.canTransform(fixed_frame_, collision_object.header.frame_id, ros::Time(0), ros::Duration(1.0)))
  {
    ROS_WARN_NAMED(LOGNAME, "Transform collision object frame '%s' to fixed frame '%s' not available after 1 second",
                   collision_object.header.frame_id.c_str(), fixed_frame_.c_str());
    return;
  }

  for (uint it_primitive = 0; it_primitive < collision_object.primitives.size(); ++it_primitive)
  {
    contours[it_primitive].resize(4);

    // safety check to make sure we process only box obstacles (all other primitives are not supported)
    if (collision_object.primitives[it_primitive].type != shape_msgs::SolidPrimitive::BOX)
    {
      ROS_WARN_NAMED(LOGNAME, "One of the obstacles received on the topic was not a box, skipping drawing it in map");
      continue;
    }

    // transform the box to the map frame
    double length = collision_object.primitives[it_primitive].dimensions[0] + length_padding * 2;
    double width = collision_object.primitives[it_primitive].dimensions[1] + width_padding * 2;
    double height = collision_object.primitives[it_primitive].dimensions[2];

    // generate poses with corners of the box
    std::vector<geometry_msgs::PoseStamped> corner_poses(4);

    // Consider primitive rotation (just yaw)
    double yaw = tf2::getYaw(collision_object.primitive_poses[it_primitive].orientation);
    double xcos, ycos, xsin, ysin, sin_yaw = sin(yaw), cos_yaw = cos(yaw);
    size_t i = 0;
    for (double y = -0.5; y <= 0.5; ++y)
    {
      for (double x = -0.5; x <= 0.5; ++x)
      {
        xcos = x * length * cos_yaw, xsin = x * length * sin_yaw;
        ycos = y * width * cos_yaw, ysin = y * width * sin_yaw;
        corner_poses[i].pose = collision_object.primitive_poses[it_primitive];
        corner_poses[i].pose.position.x = corner_poses[i].pose.position.x - (xcos - ysin);
        corner_poses[i].pose.position.y = corner_poses[i].pose.position.y - (xsin + ycos);
        corner_poses[i].pose.position.z = corner_poses[i].pose.position.z - 0.5 * height;
        corner_poses[i].header.frame_id = collision_object.header.frame_id;
        corner_poses[i].header.stamp = ros::Time(0);

        tf_.transform(corner_poses[i], contours[it_primitive][i], fixed_frame_);

        ++i;
      }
    }
    // correctly order the box so it can be drawn
    std::swap(contours[it_primitive][1], contours[it_primitive][2]);
    std::swap(contours[it_primitive][2], contours[it_primitive][3]);
  }
}

}  // namespace thorp_costmap_layers
