#pragma once

#include <moveit_msgs/CollisionObject.h>

#include <thorp_costmap_layers/UpdateObjects.h>

#include "thorp_costmap_layers/base_interface.hpp"

namespace thorp::costmap_layers
{

class ServiceInterface : public BaseInterface
{
public:
  ServiceInterface(ros::NodeHandle& nh, tf2_ros::Buffer& tf, costmap_2d::LayeredCostmap* layered_costmap);
  ~ServiceInterface();

  void reconfigure(const thorp_costmap_layers::SemanticLayerConfig& config) override;

  bool updateObjects(thorp_costmap_layers::UpdateObjects::Request& request,
                     thorp_costmap_layers::UpdateObjects::Response& response);

private:
  void objectToContour(const thorp_costmap_layers::Object& object, std::vector<geometry_msgs::PoseStamped>& contour,
                       double length_padding, double width_padding) const;

  ros::ServiceServer update_srv_;

  /// Module name for logging
  static constexpr char LOGNAME[] = "srv_interface";
};

}  // namespace thorp::costmap_layers
