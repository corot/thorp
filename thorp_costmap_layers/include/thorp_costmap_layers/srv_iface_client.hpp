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
  static ServiceClient& instance();

  bool addObject(const std::string& name, const std::string& type, const geometry_msgs::PoseStamped& pose,
                 const geometry_msgs::Vector3& size, const std::string& costmap = "both");

  bool removeObject(const std::string& name, const std::string& type, const std::string& costmap = "both");

  std::vector<thorp_costmap_layers::Object> queryObjects(const geometry_msgs::Point& lower_left,
                                                         const geometry_msgs::Point& upper_right,
                                                         const std::string& costmap = "both");

private:
  ServiceClient();

  bool callUpdateSrv(ros::ServiceClient& srv, const thorp_costmap_layers::Object& obj, const std::string& costmap);

  bool callQuerySrv(ros::ServiceClient& srv, const geometry_msgs::Point& lower_left,
                    const geometry_msgs::Point& upper_right, std::vector<thorp_costmap_layers::Object>& objects,
                    const std::string& costmap);

  ros::NodeHandle nh_;
  ros::ServiceClient lcm_qo_srv_;
  ros::ServiceClient gcm_qo_srv_;
  ros::ServiceClient lcm_uo_srv_;
  ros::ServiceClient gcm_uo_srv_;

  // Disable copy and assignment
  ServiceClient(const ServiceClient&) = delete;
  ServiceClient& operator=(const ServiceClient&) = delete;
};

}  // namespace thorp::costmap_layers
