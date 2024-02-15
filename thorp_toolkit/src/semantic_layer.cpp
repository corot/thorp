#include "thorp_toolkit/semantic_layer.hpp"

#include <thorp_msgs/UpdateCollisionObjs.h>

#include <rr_geometry/converters.hpp>
namespace rrg = rapyuta::geometry_2d;

namespace rr::nav::semantic_layer
{
ServiceClient::ServiceClient()
{
  lcm_sl_srv_ =
      nh_.serviceClient<thorp_msgs::UpdateCollisionObjs>("move_base_flex/local_costmap/semantic/update_objects");
  gcm_sl_srv_ =
      nh_.serviceClient<thorp_msgs::UpdateCollisionObjs>("move_base_flex/global_costmap/semantic/update_objects");

  if (!lcm_sl_srv_.waitForExistence(ros::Duration(10.0)) || !gcm_sl_srv_.waitForExistence(ros::Duration(1.0)))
  {
    ROS_ERROR("Failed to find the semantic layer's update_objects service");
  }
}

bool ServiceClient::addObject(const std::string& name, const std::string& type, const geometry_msgs::PoseStamped& pose,
                              const geometry_msgs::Vector3& size, const std::string& costmap)
{
  rr_nav_semantic_layer::Object obj;
  obj.operation = rr_nav_semantic_layer::Object::ADD;
  obj.name = name;
  obj.type = type;
  obj.pose = pose;
  obj.dimensions = size;

  if ((costmap == "local" || costmap == "both") && !callSrv(lcm_sl_srv_, obj, "local"))
    return false;
  if ((costmap == "global" || costmap == "both") && !callSrv(gcm_sl_srv_, obj, "global"))
    return false;

  ROS_INFO("Added object %s of type %s at %s to %s costmap%s", name.c_str(), type.c_str(), rrg::toStr(pose).c_str(),
           costmap.c_str(), costmap == "both" ? "s" : "");
  return true;
}

bool ServiceClient::removeObject(const std::string& name, const std::string& type, const std::string& costmap)
{
  thorp_msgs::UpdateCollisionObjs::Request req;
  thorp_msgs::UpdateCollisionObjs::Response resp;

  rr_nav_semantic_layer::Object obj;
  obj.operation = rr_nav_semantic_layer::Object::REMOVE;
  obj.name = name;
  obj.type = type;

  if ((costmap == "local" || costmap == "both") && !callSrv(lcm_sl_srv_, obj, "local"))
    return false;
  if ((costmap == "global" || costmap == "both") && !callSrv(gcm_sl_srv_, obj, "global"))
    return false;

  ROS_INFO("Removed object %s of type %s from %s costmap%s", name.c_str(), type.c_str(), costmap.c_str(),
           costmap == "both" ? "s" : "");
  return true;
}

bool ServiceClient::callSrv(ros::ServiceClient& srv, const rr_nav_semantic_layer::Object& obj,
                            const std::string& costmap)
{
  thorp_msgs::UpdateCollisionObjs::Request req;
  thorp_msgs::UpdateCollisionObjs::Response resp;
  req.objects.push_back(obj);

  if (!srv.call(req, resp))
  {
    ROS_ERROR_STREAM("Semantic layer's update_objects service call failed");
    return false;
  }
  if (resp.code != thorp_msgs::UpdateCollisionObjs::Response::SUCCESS)
  {
    bool add = obj.operation == rr_nav_semantic_layer::Object::ADD;
    // clang-format off
    ROS_ERROR_STREAM((add ? "Add" : "Remove") << " object " << obj.name <<
                     (add ? " to " : " from ") << costmap << " costmap "
                     << "failed with error " << resp.code << ": " << resp.text);
    // clang-format on
    return false;
  }
  return true;
}

}  // namespace rr::nav::semantic_layer
