#include "thorp_costmap_layers/srv_interface.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace thorp::costmap_layers
{

ServiceInterface::ServiceInterface(ros::NodeHandle& nh, tf2_ros::Buffer& tf, costmap_2d::LayeredCostmap* lcm)
  : BaseInterface(nh, tf, lcm)
{
  // Service to add/modify/remove semantic objects to the costmap
  update_srv_ = nh_.advertiseService("update_objects", &ServiceInterface::updateObjects, this);
}

ServiceInterface::~ServiceInterface() = default;

void ServiceInterface::reconfigure(const thorp_costmap_layers::SemanticLayerConfig& config)
{
  enabled_ = config.service_enabled;
}

bool ServiceInterface::updateObjects(thorp_costmap_layers::UpdateObjects::Request& request,
                                     thorp_costmap_layers::UpdateObjects::Response& response)
{
  response.code = thorp_costmap_layers::UpdateObjects::Response::SUCCESS;
  response.text = "Object successfully updated";
  bool trigger_map_update = false;
  for (const auto& obj : request.objects)
  {
    // Discard objects of unknown type
    if (object_types_.find(obj.type) == object_types_.end())
    {
      ROS_WARN_NAMED(LOGNAME, "Discarded object %s of unknown type %s", obj.name.c_str(), obj.type.c_str());
      response.code = thorp_costmap_layers::UpdateObjects::Response::INVALID_OBJECT_TYPE;
      response.text = obj.type;
      break;
    }
    const ObjectType& type = object_types_[obj.type];
    if (obj.operation == thorp_costmap_layers::Object::ADD)
    {
      // transform obstacle to correct frame
      std::vector<std::vector<geometry_msgs::PoseStamped>> contours(1);
      objectToContour(obj, contours[0], type.length_padding, type.width_padding);
      // convert into obstacles and save into hash -> also inserts them into updated
      contoursToHash(contours, obj.name, obj.type);

      ROS_DEBUG_NAMED(LOGNAME, "Add object %s of type %s", obj.name.c_str(), obj.type.c_str());
    }
    else if (obj.operation == thorp_costmap_layers::Object::REMOVE)
    {
      if (removeContours(obj.name))
      {
        ROS_DEBUG_NAMED(LOGNAME, "Removed object %s of type %s", obj.name.c_str(), obj.type.c_str());
      }
      else
      {
        ROS_WARN_NAMED(LOGNAME, "Attempting to remove unknown object %s", obj.name.c_str());
        response.code = thorp_costmap_layers::UpdateObjects::Response::OBJECT_NOT_FOUND;
        response.text = obj.name;
        break;
      }
    }
    
    if (type.force_update)
      trigger_map_update = true;
  }
  if (trigger_map_update)
  {
    // one or more added/removed objects require immediate update: force update instead of waiting for update thread
    layered_costmap_->updateMap(robot_x_, robot_y_, robot_yaw_);
  }
  return response.code == thorp_costmap_layers::UpdateObjects::Response::SUCCESS;
}

// transform primitive obstacle to current frame of db
void ServiceInterface::objectToContour(const thorp_costmap_layers::Object& obj,
                                       std::vector<geometry_msgs::PoseStamped>& contour, double length_padding,
                                       double width_padding) const
{
  // we should be able to transform obstacles instantly
  if (!tf_.canTransform(fixed_frame_, obj.pose.header.frame_id, ros::Time(0), ros::Duration(1.0)))
  {
    ROS_WARN_NAMED(LOGNAME, "Transform object frame '%s' to fixed frame '%s' not available after 1 second",
                   obj.pose.header.frame_id.c_str(), fixed_frame_.c_str());
    return;
  }
  
  contour.resize(4);

  // transform the box to the map frame
  const double length = obj.dimensions.x + length_padding * 2;
  const double width = obj.dimensions.y + width_padding * 2;
  const double height = obj.dimensions.z;

  // generate poses with corners of the box
  std::vector<geometry_msgs::PoseStamped> corner_poses(4);

  // Consider primitive rotation (just yaw)
  const double yaw = tf2::getYaw(obj.pose.pose.orientation);
  const double sin_yaw = sin(yaw), cos_yaw = cos(yaw);
  double xcos, ycos, xsin, ysin;
  size_t i = 0;
  for (double y = -0.5; y <= 0.5; ++y)
  {
    for (double x = -0.5; x <= 0.5; ++x)
    {
      xcos = x * length * cos_yaw, xsin = x * length * sin_yaw;
      ycos = y * width * cos_yaw, ysin = y * width * sin_yaw;
      corner_poses[i] = obj.pose;
      corner_poses[i].pose.position.x = corner_poses[i].pose.position.x - (xcos - ysin);
      corner_poses[i].pose.position.y = corner_poses[i].pose.position.y - (xsin + ycos);
      corner_poses[i].pose.position.z = corner_poses[i].pose.position.z - 0.5 * height;

      tf_.transform(corner_poses[i], contour[i], fixed_frame_);

      ++i;
    }
  }
  // correctly order the box so it can be drawn
  std::swap(contour[1], contour[2]);
  std::swap(contour[2], contour[3]);
}

}  // namespace thorp::costmap_layers
