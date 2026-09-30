#pragma once

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "thorp_bt_cpp/service_call.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <thorp_costmap_layers/srv/query_objects.hpp>
#include <thorp_costmap_layers/srv/update_objects.hpp>

namespace thorp::bt
{
/** A service of a costmap's semantic layer */
inline std::string semanticLayerService(const std::string& costmap, const std::string& service)
{
  return "/" + costmap + "_costmap/" + costmap + "_costmap/semantic_layer/" + service;
}

/**
 * Add or remove objects on a costmap's semantic layer; false if it fails, logging why.
 */
inline bool updateSemanticLayer(const rclcpp::Node::SharedPtr& node, const std::string& costmap,
                                const std::vector<thorp_costmap_layers::msg::Object>& objects)
{
  using thorp_costmap_layers::srv::UpdateObjects;
  UpdateObjects::Request request;
  request.objects = objects;
  const auto response = callService<UpdateObjects>(node, semanticLayerService(costmap, "update_objects"), request);
  if (!response)
    return false;
  if (response->code != UpdateObjects::Response::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Updating %s costmap's semantic layer failed with code %d: %s", costmap.c_str(),
                 response->code, response->text.c_str());
    return false;
  }
  return true;
}

/**
 * The objects on a costmap's semantic layer overlapping a region of the map; none if the query fails, logging why.
 */
inline std::optional<std::vector<thorp_costmap_layers::msg::Object>>
querySemanticLayer(const rclcpp::Node::SharedPtr& node, const std::string& costmap,
                   const geometry_msgs::msg::Point& lower_left, const geometry_msgs::msg::Point& upper_right)
{
  using thorp_costmap_layers::srv::QueryObjects;
  QueryObjects::Request request;
  request.lower_left = lower_left;
  request.upper_right = upper_right;
  const auto response = callService<QueryObjects>(node, semanticLayerService(costmap, "query_objects"), request);
  if (!response)
    return std::nullopt;
  return response->objects;
}

}  // namespace thorp::bt
