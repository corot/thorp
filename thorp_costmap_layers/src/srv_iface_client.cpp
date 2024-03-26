#include "thorp_costmap_layers/srv_iface_client.hpp"

#include <thorp_costmap_layers/QueryObjects.h>
#include <thorp_costmap_layers/UpdateObjects.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::costmap_layers
{

ServiceClient& ServiceClient::instance()
{
  static ServiceClient instance;
  return instance;
}

ServiceClient::ServiceClient()
{
  lcm_qo_srv_ = nh_.serviceClient<thorp_costmap_layers::QueryObjects>(
      "move_base_flex/local_costmap/semantic_layer/query_objects");
  gcm_qo_srv_ = nh_.serviceClient<thorp_costmap_layers::QueryObjects>(
      "move_base_flex/global_costmap/semantic_layer/query_objects");
  lcm_uo_srv_ = nh_.serviceClient<thorp_costmap_layers::UpdateObjects>(
      "move_base_flex/local_costmap/semantic_layer/update_objects");
  gcm_uo_srv_ = nh_.serviceClient<thorp_costmap_layers::UpdateObjects>(
      "move_base_flex/global_costmap/semantic_layer/update_objects");

  if (!lcm_qo_srv_.waitForExistence(ros::Duration(30.0)) || !gcm_qo_srv_.waitForExistence(ros::Duration(30.0)) ||
      !lcm_uo_srv_.waitForExistence(ros::Duration(30.0)) || !gcm_uo_srv_.waitForExistence(ros::Duration(30.0)))
  {
    ROS_ERROR("Failed to connect to the semantic layer's services");
  }
}

bool ServiceClient::addObject(const std::string& name, const std::string& type, const geometry_msgs::PoseStamped& pose,
                              const geometry_msgs::Vector3& size, const std::string& costmap)
{
  thorp_costmap_layers::Object obj;
  obj.operation = thorp_costmap_layers::Object::ADD;
  obj.name = name;
  obj.type = type;
  obj.pose = pose;
  obj.dimensions = size;

  if ((costmap == "local" || costmap == "both") && !callUpdateSrv(lcm_uo_srv_, obj, "local"))
    return false;
  if ((costmap == "global" || costmap == "both") && !callUpdateSrv(gcm_uo_srv_, obj, "global"))
    return false;

  ROS_INFO("Added object %s of type %s at %s to %s costmap%s", name.c_str(), type.c_str(), ttk::toCStr2D(pose),
           costmap.c_str(), costmap == "both" ? "s" : "");
  return true;
}

bool ServiceClient::removeObject(const std::string& name, const std::string& type, const std::string& costmap)
{
  thorp_costmap_layers::UpdateObjects::Request req;
  thorp_costmap_layers::UpdateObjects::Response resp;

  thorp_costmap_layers::Object obj;
  obj.operation = thorp_costmap_layers::Object::REMOVE;
  obj.name = name;
  obj.type = type;

  if ((costmap == "local" || costmap == "both") && !callUpdateSrv(lcm_uo_srv_, obj, "local"))
    return false;
  if ((costmap == "global" || costmap == "both") && !callUpdateSrv(gcm_uo_srv_, obj, "global"))
    return false;

  ROS_INFO("Removed object %s of type %s from %s costmap%s", name.c_str(), type.c_str(), costmap.c_str(),
           costmap == "both" ? "s" : "");
  return true;
}

std::vector<thorp_costmap_layers::Object> ServiceClient::queryObjects(const geometry_msgs::Point& lower_left,
                                                                      const geometry_msgs::Point& upper_right,
                                                                      const std::string& costmap)
{
  std::vector<thorp_costmap_layers::Object> objects;

  if ((costmap == "local" || costmap == "both") &&
      !callQuerySrv(lcm_qo_srv_, lower_left, upper_right, objects, "local"))
    return {};
  if ((costmap == "global" || costmap == "both") &&
      !callQuerySrv(gcm_qo_srv_, lower_left, upper_right, objects, "global"))
    return {};
  return objects;
}

bool ServiceClient::callUpdateSrv(ros::ServiceClient& srv, const thorp_costmap_layers::Object& obj,
                                  const std::string& costmap)
{
  thorp_costmap_layers::UpdateObjects::Request req;
  thorp_costmap_layers::UpdateObjects::Response resp;
  req.objects.push_back(obj);

  if (!srv.call(req, resp))
  {
    ROS_ERROR_STREAM("Semantic layer's update_objects service call failed");
    return false;
  }
  if (resp.code != thorp_costmap_layers::UpdateObjects::Response::SUCCESS)
  {
    bool add = obj.operation == thorp_costmap_layers::Object::ADD;
    // clang-format off
    ROS_ERROR_STREAM((add ? "Add" : "Remove") << " object " << obj.name <<
                     (add ? " to " : " from ") << costmap << " costmap "
                     << "failed with error " << resp.code << ": " << resp.text);
    // clang-format on
    return false;
  }
  return true;
}

bool ServiceClient::callQuerySrv(ros::ServiceClient& srv, const geometry_msgs::Point& lower_left,
                                 const geometry_msgs::Point& upper_right,

                                 std::vector<thorp_costmap_layers::Object>& objects, const std::string& costmap)
{
  thorp_costmap_layers::QueryObjects::Request req;
  req.lower_left = lower_left;
  req.upper_right = upper_right;
  thorp_costmap_layers::QueryObjects::Response resp;
  if (!srv.call(req, resp))
  {
    ROS_ERROR_STREAM("Semantic layer's query_objects service call failed");
    return false;
  }
  objects.insert(objects.end(), resp.objects.begin(), resp.objects.end());
  return true;
}

}  // namespace thorp::costmap_layers
