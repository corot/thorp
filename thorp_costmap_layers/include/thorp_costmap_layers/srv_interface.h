#pragma once

#include <moveit_msgs/CollisionObject.h>

#include <thorp_msgs/UpdateCollisionObjs.h>

#include "thorp_costmap_layers/base_interface.h"


namespace thorp_costmap_layers
{

class ServiceInterface : public BaseInterface
{
public:
  ServiceInterface(ros::NodeHandle& nh, tf2_ros::Buffer& tf, const std::string& map_frame,
                   std::function<void(double, double, double)> update_map_callback);
  ~ServiceInterface();

  bool updateCollisionObjs(thorp_msgs::UpdateCollisionObjs::Request& request,
                           thorp_msgs::UpdateCollisionObjs::Response& response);

  bool getCallbackProcessed() { return callback_processed_; }

private:
  void collisionObjToContours(const moveit_msgs::CollisionObject& collision_object,
                              std::vector<std::vector<geometry_msgs::PoseStamped>>& contours,
                              double length_padding, double width_padding) const;

  ros::ServiceServer update_srv_;

  bool callback_processed_;
  std::function<void(double, double, double)> update_map_callback_;
};

}
