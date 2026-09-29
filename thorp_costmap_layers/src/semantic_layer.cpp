#include "thorp_costmap_layers/semantic_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::MapLocation;

namespace thorp_costmap_layers
{
namespace
{
/**
 * Clip a segment to a rectangle (Liang-Barsky), so objects partially outside the costmap don't get fake edges at its
 * borders. False if the segment is entirely outside.
 */
bool clipSegment(double left, double right, double bottom, double top, double& x0, double& y0, double& x1, double& y1)
{
  double t0 = 0.0;
  double t1 = 1.0;
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const std::array<double, 4> p{ -dx, dx, -dy, dy };
  const std::array<double, 4> q{ x0 - left, right - x0, y0 - bottom, top - y0 };
  for (size_t edge = 0; edge < 4; ++edge)
  {
    if (p[edge] == 0.0)
    {
      if (q[edge] < 0.0)
        return false;  // parallel to this edge, and outside
      continue;
    }
    const double r = q[edge] / p[edge];
    if (p[edge] < 0.0)
    {
      if (r > t1)
        return false;
      t0 = std::max(t0, r);
    }
    else
    {
      if (r < t0)
        return false;
      t1 = std::min(t1, r);
    }
  }
  x1 = x0 + t1 * dx;
  y1 = y0 + t1 * dy;
  x0 = x0 + t0 * dx;
  y0 = y0 + t0 * dy;
  return true;
}
}  // namespace

void SemanticLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node)
    throw std::runtime_error("Failed to lock node");

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("fixed_frame", rclcpp::ParameterValue(fixed_frame_));
  declareParameter("object_types", rclcpp::ParameterValue(std::vector<std::string>{}));
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("fixed_frame"), fixed_frame_);
  std::vector<std::string> type_names;
  node->get_parameter(getFullName("object_types"), type_names);
  for (const auto& type_name : type_names)
  {
    ObjectType type;
    declareParameter(type_name + ".cost", rclcpp::ParameterValue(type.cost));
    declareParameter(type_name + ".fill", rclcpp::ParameterValue(type.fill));
    declareParameter(type_name + ".length_padding", rclcpp::ParameterValue(type.length_padding));
    declareParameter(type_name + ".width_padding", rclcpp::ParameterValue(type.width_padding));
    declareParameter(type_name + ".precedence", rclcpp::ParameterValue(type.precedence));
    declareParameter(type_name + ".use_maximum", rclcpp::ParameterValue(type.use_maximum));
    node->get_parameter(getFullName(type_name + ".cost"), type.cost);
    node->get_parameter(getFullName(type_name + ".fill"), type.fill);
    node->get_parameter(getFullName(type_name + ".length_padding"), type.length_padding);
    node->get_parameter(getFullName(type_name + ".width_padding"), type.width_padding);
    node->get_parameter(getFullName(type_name + ".precedence"), type.precedence);
    node->get_parameter(getFullName(type_name + ".use_maximum"), type.use_maximum);
    object_types_[type_name] = type;
    RCLCPP_INFO(logger_, "%s: object type %s: cost %g, fill %d, padding %g x %g, precedence %d", name_.c_str(),
                type_name.c_str(), type.cost, type.fill, type.length_padding, type.width_padding, type.precedence);
  }

  matchSize();
  current_ = true;

  update_srv_ = node->create_service<srv::UpdateObjects>(
      "~/" + name_ + "/update_objects",
      [this](const std::shared_ptr<srv::UpdateObjects::Request> request,
             std::shared_ptr<srv::UpdateObjects::Response> response) { updateObjects(request, response); },
      rclcpp::ServicesQoS(), callback_group_);
  query_srv_ = node->create_service<srv::QueryObjects>(
      "~/" + name_ + "/query_objects",
      [this](const std::shared_ptr<srv::QueryObjects::Request> request,
             std::shared_ptr<srv::QueryObjects::Response> response) { queryObjects(request, response); },
      rclcpp::ServicesQoS(), callback_group_);
}

void SemanticLayer::updateObjects(const std::shared_ptr<srv::UpdateObjects::Request> request,
                                  std::shared_ptr<srv::UpdateObjects::Response> response)
{
  using srv::UpdateObjects;
  response->code = UpdateObjects::Response::SUCCESS;
  for (const auto& msg : request->objects)
  {
    const auto type = object_types_.find(msg.type);
    if (type == object_types_.end())
    {
      RCLCPP_WARN(logger_, "%s: object %s of unknown type %s", name_.c_str(), msg.name.c_str(), msg.type.c_str());
      response->code = UpdateObjects::Response::INVALID_OBJECT_TYPE;
      response->text = msg.type;
      return;
    }

    if (msg.operation == msg::Object::ADD)
    {
      const auto object = makeObject(msg, type->second);
      if (!object)
      {
        response->code = UpdateObjects::Response::INVALID_TARGET_POSE;
        response->text = msg.name;
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      const auto previous = objects_.find(msg.name);
      if (previous != objects_.end())
        removed_boxes_.push_back(previous->second.bounding_box);
      objects_[msg.name] = *object;
      RCLCPP_INFO(logger_, "%s: added %s %s", name_.c_str(), msg.type.c_str(), msg.name.c_str());
    }
    else if (msg.operation == msg::Object::REMOVE)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto object = objects_.find(msg.name);
      if (object == objects_.end())
      {
        RCLCPP_WARN(logger_, "%s: cannot remove unknown object %s", name_.c_str(), msg.name.c_str());
        response->code = UpdateObjects::Response::OBJECT_NOT_FOUND;
        response->text = msg.name;
        return;
      }
      removed_boxes_.push_back(object->second.bounding_box);
      objects_.erase(object);
      RCLCPP_INFO(logger_, "%s: removed %s %s", name_.c_str(), msg.type.c_str(), msg.name.c_str());
    }
  }
}

void SemanticLayer::queryObjects(const std::shared_ptr<srv::QueryObjects::Request> request,
                                 std::shared_ptr<srv::QueryObjects::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& [name, object] : objects_)
  {
    const auto& box = object.bounding_box;
    if (box[0] <= request->upper_right.x && box[2] >= request->lower_left.x && box[1] <= request->upper_right.y &&
        box[3] >= request->lower_left.y)
      response->objects.push_back(object.msg);
  }
}

std::optional<SemanticLayer::Object> SemanticLayer::makeObject(const msg::Object& msg, const ObjectType& type) const
{
  geometry_msgs::msg::PoseStamped pose = msg.pose;
  pose.header.stamp = rclcpp::Time(0);  // latest transform; objects are kept on the fixed frame since added
  try
  {
    pose = tf_->transform(pose, fixed_frame_, tf2::durationFromSec(1.0));
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_WARN(logger_, "%s: cannot transform object %s to %s: %s", name_.c_str(), msg.name.c_str(),
                fixed_frame_.c_str(), e.what());
    return std::nullopt;
  }

  Object object;
  object.msg = msg;
  object.bounding_box = { std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest() };
  const double half_length = msg.dimensions.x / 2.0 + type.length_padding;
  const double half_width = msg.dimensions.y / 2.0 + type.width_padding;
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const std::array<std::pair<double, double>, 4> local_corners{ { { half_length, half_width },
                                                                   { -half_length, half_width },
                                                                   { -half_length, -half_width },
                                                                   { half_length, -half_width } } };
  for (size_t i = 0; i < local_corners.size(); ++i)
  {
    const auto& [x, y] = local_corners[i];
    object.corners[2 * i] = pose.pose.position.x + cos_yaw * x - sin_yaw * y;
    object.corners[2 * i + 1] = pose.pose.position.y + sin_yaw * x + cos_yaw * y;
    object.bounding_box[0] = std::min(object.bounding_box[0], object.corners[2 * i]);
    object.bounding_box[1] = std::min(object.bounding_box[1], object.corners[2 * i + 1]);
    object.bounding_box[2] = std::max(object.bounding_box[2], object.corners[2 * i]);
    object.bounding_box[3] = std::max(object.bounding_box[3], object.corners[2 * i + 1]);
  }
  return object;
}

void SemanticLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double* min_x,
                                 double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  geometry_msgs::msg::TransformStamped to_costmap;
  try
  {
    to_costmap = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), fixed_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_WARN(logger_, "%s: %s; skipping update", name_.c_str(), e.what());
    return;
  }

  // Every object, on every update, so the rolling local costmap paints those entering it
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& [name, object] : objects_)
    touchBox(to_costmap, object.bounding_box, min_x, min_y, max_x, max_y);
  for (const auto& box : removed_boxes_)
    touchBox(to_costmap, box, min_x, min_y, max_x, max_y);
  removed_boxes_.clear();
}

void SemanticLayer::touchBox(const geometry_msgs::msg::TransformStamped& to_costmap, const std::array<double, 4>& box,
                             double* min_x, double* min_y, double* max_x, double* max_y)
{
  for (const auto& [x, y] : { std::pair{ box[0], box[1] }, std::pair{ box[0], box[3] }, std::pair{ box[2], box[1] },
                              std::pair{ box[2], box[3] } })
  {
    geometry_msgs::msg::Point corner, corner_transformed;
    corner.x = x;
    corner.y = y;
    tf2::doTransform(corner, corner_transformed, to_costmap);
    touch(corner_transformed.x, corner_transformed.y, min_x, min_y, max_x, max_y);
  }
}

void SemanticLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  geometry_msgs::msg::TransformStamped to_costmap;
  try
  {
    to_costmap = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), fixed_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException& e)
  {
    return;  // already reported by updateBounds
  }

  // The updated area, in world coordinates on the costmap frame
  const double left = master_grid.getOriginX() + min_i * master_grid.getResolution();
  const double right = master_grid.getOriginX() + max_i * master_grid.getResolution();
  const double bottom = master_grid.getOriginY() + min_j * master_grid.getResolution();
  const double top = master_grid.getOriginY() + max_j * master_grid.getResolution();

  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<const Object*> objects;
  for (const auto& [name, object] : objects_)
    objects.push_back(&object);
  std::stable_sort(objects.begin(), objects.end(), [this](const Object* a, const Object* b) {
    return object_types_.at(a->msg.type).precedence < object_types_.at(b->msg.type).precedence;
  });

  for (const Object* object : objects)
  {
    const ObjectType& type = object_types_.at(object->msg.type);
    std::array<double, 8> corners;
    for (size_t i = 0; i < 4; ++i)
    {
      geometry_msgs::msg::Point corner, corner_transformed;
      corner.x = object->corners[2 * i];
      corner.y = object->corners[2 * i + 1];
      tf2::doTransform(corner, corner_transformed, to_costmap);
      corners[2 * i] = corner_transformed.x;
      corners[2 * i + 1] = corner_transformed.y;
    }

    // Each side clipped to the updated area; filled objects take the clipped points as the polygon to fill
    std::vector<MapLocation> cells;
    std::vector<MapLocation> fill_polygon;
    PolygonOutlineCells outline(*this, costmap_, cells);
    for (size_t i = 0; i < 4; ++i)
    {
      double x0 = corners[2 * i], y0 = corners[2 * i + 1];
      double x1 = corners[(2 * i + 2) % 8], y1 = corners[(2 * i + 3) % 8];
      if (!clipSegment(left, right, bottom, top, x0, y0, x1, y1))
        continue;
      int mx0, my0, mx1, my1;
      master_grid.worldToMapEnforceBounds(x0, y0, mx0, my0);
      master_grid.worldToMapEnforceBounds(x1, y1, mx1, my1);
      if (type.fill)
      {
        fill_polygon.push_back({ static_cast<unsigned int>(mx0), static_cast<unsigned int>(my0) });
        fill_polygon.push_back({ static_cast<unsigned int>(mx1), static_cast<unsigned int>(my1) });
      }
      else
      {
        raytraceLine(outline, mx0, my0, mx1, my1);
      }
    }
    if (type.fill && fill_polygon.size() >= 3)
      convexFillCells(fill_polygon, cells);

    const auto cost = static_cast<unsigned char>(std::round(type.cost * LETHAL_OBSTACLE));
    for (const auto& cell : cells)
    {
      const int x = static_cast<int>(cell.x);
      const int y = static_cast<int>(cell.y);
      if (x >= min_i && x < max_i && y >= min_j && y < max_j &&
          (!type.use_maximum || master_grid.getCost(x, y) < cost))
        master_grid.setCost(x, y, cost);
    }
  }
}

}  // namespace thorp_costmap_layers

PLUGINLIB_EXPORT_CLASS(thorp_costmap_layers::SemanticLayer, nav2_costmap_2d::Layer)
