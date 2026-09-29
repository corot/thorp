#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "thorp_bt_cpp/service_call.hpp"

#include <thorp_costmap_layers/srv/update_objects.hpp>

namespace thorp::bt
{
/**
 * Add or remove objects on a costmap's semantic layer; false if it fails, logging why.
 */
inline bool updateSemanticLayer(const rclcpp::Node::SharedPtr& node, const std::string& costmap,
                                const std::vector<thorp_costmap_layers::msg::Object>& objects)
{
  using thorp_costmap_layers::srv::UpdateObjects;
  UpdateObjects::Request request;
  request.objects = objects;
  const auto response = callService<UpdateObjects>(
      node, "/" + costmap + "_costmap/" + costmap + "_costmap/semantic_layer/update_objects", request);
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

}  // namespace thorp::bt
