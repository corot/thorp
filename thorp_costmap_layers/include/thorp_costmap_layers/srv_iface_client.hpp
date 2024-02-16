#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include "thorp_costmap_layers/Object.h"

namespace thorp::costmap_layers
{
class ServiceClient
{
public:
  ServiceClient();

  bool addObject(const std::string& name, const std::string& type, const geometry_msgs::PoseStamped& pose,
                 const geometry_msgs::Vector3& size, const std::string& costmap = "both");

  bool removeObject(const std::string& name, const std::string& type, const std::string& costmap = "both");

private:
  bool callSrv(ros::ServiceClient& srv, const thorp_costmap_layers::Object& obj, const std::string& costmap);

  ros::NodeHandle nh_;
  ros::ServiceClient lcm_sl_srv_;
  ros::ServiceClient gcm_sl_srv_;
};

}  // namespace thorp::costmap_layers
