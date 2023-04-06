#pragma once

#include <moveit_msgs/CollisionObject.h>

#include <thorp_msgs/UpdateCollisionObjs.h>

#include "thorp_costmap_layers/base_interface.hpp"


namespace thorp_costmap_layers
{

class ServiceInterface : public BaseInterface
{
public:
  ServiceInterface(ros::NodeHandle& nh, tf2_ros::Buffer& tf, costmap_2d::LayeredCostmap* layered_costmap);
  ~ServiceInterface();

  void reconfigure(const thorp_costmap_layers::SemanticLayerConfig& config);

  bool updateCollisionObjs(thorp_msgs::UpdateCollisionObjs::Request& request,
                           thorp_msgs::UpdateCollisionObjs::Response& response);

private:
  void collisionObjToContours(const moveit_msgs::CollisionObject& collision_object,
                              std::vector<std::vector<geometry_msgs::PoseStamped>>& contours,
                              double length_padding, double width_padding) const;

  ros::ServiceServer update_srv_;

  /// Module name for logging
  static constexpr char LOGNAME[] = "srv_interface";
};

}  // namespace thorp_costmap_layers
