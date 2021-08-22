#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;

#include "thorp_costmap_layers/base_interface.h"


namespace thorp_costmap_layers
{

BaseInterface::BaseInterface(ros::NodeHandle& nh, tf2_ros::Buffer& tf, const std::string& map_frame) :
  nh_(nh),
  tf_(tf),
  map_frame_(map_frame),
  primitives_count_(),
  object_types_(),
  ids_removed_(),
  ids_updated_(),
  spatial_hash_(0.5)
{
  // Read object types configuration
  XmlRpc::XmlRpcValue object_types;
  nh_.param("object_types", object_types, object_types);
  if (object_types.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN_STREAM("[scene_interface] Parameter 'object_types' must contain a list of dictionaries; nothing to do");
    return;  // do not subscribe to scene topic, as we have no idea of what to do with the received information
  }

  for (int32_t i = 0; i < object_types.size(); ++i)
  {
    if (object_types[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN_STREAM("[scene_interface] Parameter 'object_types' must contain a list of dictionaries; nothing to do");
      return;  // do not subscribe to scene topic, as we have no idea of what to do with the received information
    }
    std::string type = object_types[i]["type"];  // type is the only mandatory field
    if (object_types[i]["cost"].valid())
      object_types_[type].cost = static_cast<double>(object_types[i]["cost"]);
    if (object_types[i]["fill"].valid())
      object_types_[type].fill = static_cast<bool>(object_types[i]["fill"]);
    if (object_types[i]["precedence"].valid())
      object_types_[type].precedence = static_cast<int>(object_types[i]["precedence"]);
    if (object_types[i]["use_maximum"].valid())
      object_types_[type].use_maximum = static_cast<bool>(object_types[i]["use_maximum"]);
    if (object_types[i]["force_update"].valid())
      object_types_[type].force_update = static_cast<bool>(object_types[i]["force_update"]);
    if (object_types[i]["width_padding"].valid())
      object_types_[type].width_padding = static_cast<double>(object_types[i]["width_padding"]);
    if (object_types[i]["length_padding"].valid())
      object_types_[type].length_padding = static_cast<double>(object_types[i]["length_padding"]);
    ROS_INFO("Object type %s: cost = %g, fill = %d, padding = %g / %g", type.c_str(), object_types_[type].cost,
             object_types_[type].fill, object_types_[type].width_padding, object_types_[type].length_padding);
  }
}

BaseInterface::~BaseInterface()
{
}

std::set<std::shared_ptr<Object>> BaseInterface::getUpdatedObjects()
{
  updated_mutex_.lock();
  std::set<std::shared_ptr<Object>> updated_objs;
  for (const auto id_updated : ids_updated_)
  {
    std::shared_ptr<Object> updated_obj = spatial_hash_.getObject(id_updated);
    if (updated_obj == nullptr)
    {
      ROS_WARN_STREAM("[base interface] An updated object with id: " << id_updated << " is not in the hash!");
      continue;
    }
    updated_objs.insert(updated_obj);
  }

  return updated_objs;
}

std::set<std::shared_ptr<Object>> BaseInterface::getRemovedObjects()
{
  removed_mutex_.lock();
  std::set<std::shared_ptr<Object>> removed_objs;
  for (const auto id_removed : ids_removed_)
  {
    std::shared_ptr<Object> removed_obj = spatial_hash_.getObject(id_removed);
    if (removed_obj == nullptr)
    {
      ROS_WARN_STREAM("[semantic layer] A removed object with id: " << id_removed << " is not in the hash!");
      continue;
    }
    removed_objs.insert(spatial_hash_.getObject(id_removed));
  }

  return removed_objs;
}

std::vector<Object> BaseInterface::getObjectsInRegion(const Point2d& lower_left,
                                                      const Point2d& upper_right) const
{
  return spatial_hash_.getObjectsInRegion(lower_left, upper_right);
}

std::shared_ptr<Object> BaseInterface::getObject(const int id) const
{
  return spatial_hash_.getObject(id);
}

int BaseInterface::makeHash(const std::string &id, int primitive) const
{
  return std::hash<std::string> {}(id + "," + std::to_string(primitive));
}

void BaseInterface::contoursToHash(const std::vector<std::vector<geometry_msgs::PoseStamped>>& contours,
                                   const std::string& id, const ObjectType& type)
{
  Object object;
  object.confirmed = true;
  object.cost = type.cost;
  object.fill = type.fill;
  object.precedence = type.precedence;
  object.use_maximum = type.use_maximum;

  for (size_t i = 0; i < contours.size(); ++i)
  {
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    object.contour_points.clear();
    for (const auto& contour_point : contours[i])
    {
      // we need the bounding box to get the visible objects later during drawing
      object.contour_points.emplace_back(contour_point.pose.position.x, contour_point.pose.position.y);

      min_x = std::min(min_x, contour_point.pose.position.x);
      max_x = std::max(max_x, contour_point.pose.position.x);
      min_y = std::min(min_y, contour_point.pose.position.y);
      max_y = std::max(max_y, contour_point.pose.position.y);
    }
    object.bounding_box = Rectangle(Point2d(min_x, min_y), Point2d(max_x, max_y));
    object.id = makeHash(id, i);
    updateObject(object);
    primitives_count_[id] = contours.size();
  }
}

void BaseInterface::updateObject(const Object& object)
{
  spatial_hash_.updateObject(object);

  updated_mutex_.lock();
  ids_updated_.insert(object.id);
  updated_mutex_.unlock();

  removed_mutex_.lock();
  ids_removed_.erase(object.id);   // updated objects cannot be simultaneously removed
  removed_mutex_.unlock();
}

bool BaseInterface::removeContours(const std::string& id)
{
  try
  {
    std::lock_guard<std::mutex> lock(removed_mutex_);
    for (int it_prim = 0; it_prim < primitives_count_.at(id); ++it_prim)
    {
      ids_removed_.insert(makeHash(id, it_prim));
    }
    return primitives_count_.erase(id);
  }
  catch (const std::out_of_range& e)
  {
    return false;
  }
}

void BaseInterface::removeObject(int hash)
{
  std::lock_guard<std::mutex> lock(removed_mutex_);
  ids_removed_.insert(hash);
}

}