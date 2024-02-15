#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include "rr_nav_semantic_layer/Object.h"

namespace rr::nav::semantic_layer
{
class ServiceClient
{
public:
  ServiceClient();

  bool addObject(const std::string& name, const std::string& type, const geometry_msgs::PoseStamped& pose,
                 const geometry_msgs::Vector3& size, const std::string& costmap = "both");

  bool removeObject(const std::string& name, const std::string& type, const std::string& costmap = "both");

private:
  bool callSrv(ros::ServiceClient& srv, const rr_nav_semantic_layer::Object& obj, const std::string& costmap);

  ros::NodeHandle nh_;
  ros::ServiceClient lcm_sl_srv_;
  ros::ServiceClient gcm_sl_srv_;
};

}  // namespace rr::nav::semantic_layer
