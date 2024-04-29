/*
* Author: Jorge Santos
*/

#include "thorp_manipulation/tray_manager_server.hpp"

#include <thorp_toolkit/geometry.hpp>

namespace thorp::manipulation
{

TrayManagerServer::TrayManagerServer() : planning_scene_(ttk::PlanningScene::instance())
{
  ros::NodeHandle pnh("~");
  ros::NodeHandle tray_nh(pnh, "tray");
  tray_link_ = tray_nh.param("link", std::string("tray_link"));
  tray_slot_ = tray_nh.param("slot", 0.035);
  tray_side_x_ = tray_nh.param("side_x", 0.14);
  tray_side_y_ = tray_nh.param("side_y", 0.14);
  slots_x_ = (int)std::round(tray_side_x_ / tray_slot_ + 0.1);
  slots_y_ = (int)std::round(tray_side_y_ / tray_slot_ + 0.1);
  offset_x_ = (slots_x_ % 2 == 0) ? tray_slot_ / 2.0 : 0.0;
  offset_y_ = (slots_y_ % 2 == 0) ? tray_slot_ / 2.0 : 0.0;
  offset_z_ = pnh.param("placing_height_on_tray", 0.03);

  ROS_DEBUG_NAMED(LOGNAME, "Tray dimensions: %g x %g m. %d x %d slots of %g x %g m each", tray_side_x_, tray_side_y_,
                  slots_x_, slots_y_, tray_slot_, tray_slot_);

  planning_scene_.addObject("tray", ttk::createPose(0, 0, 0.0015, 0, 0, 0, tray_link_),
                            { tray_side_x_ + 0.01, tray_side_y_ + 0.01, 0.002 }, "green");

  clear_pl_scene_srv_ = pnh.advertiseService("clear_planning_scene", &TrayManagerServer::clearPlanningSceneCB, this);
  tray_add_object_srv_ = tray_nh.advertiseService("add_object", &TrayManagerServer::trayAddObjectCB, this);
  tray_capacity_srv_ = tray_nh.advertiseService("get_capacity", &TrayManagerServer::trayCapacityCB, this);
  tray_next_pose_srv_ = tray_nh.advertiseService("get_next_pose", &TrayManagerServer::trayNextPoseCB, this);

  visualizePlacePoses();
}

bool TrayManagerServer::clearPlanningSceneCB(thorp_msgs::ClearPlanningSceneRequest& request,
                                             thorp_msgs::ClearPlanningSceneResponse& response)
{
  std::set<std::string> exempted;
  if (request.keep_tray)
  {
    exempted.insert("tray");
    exempted.insert(objs_on_tray_.begin(), objs_on_tray_.end());
  }
  planning_scene_.removeAll(exempted);
  return true;
}

bool TrayManagerServer::trayAddObjectCB(thorp_msgs::TrayAddObjectRequest& request,
                                        thorp_msgs::TrayAddObjectResponse& response)
{
  // Move a collision object to the tray
  // we need to add half the object size, so it appears at the right height on planning scene
  geometry_msgs::PoseStamped obj_pose;
  geometry_msgs::Vector3 obj_size;
  planning_scene_.getObjectData(request.object_name, obj_pose, obj_size);  // TODO getObjectSize
  request.pose_on_tray.pose.position.z = obj_size.z / 2.0;
  ROS_INFO_NAMED(LOGNAME, "Object '%s' added to tray at %s", request.object_name.c_str(),
                 ttk::toCStr3D(request.pose_on_tray));
  planning_scene_.displaceObject(request.object_name, request.pose_on_tray);

  objs_on_tray_.push_back(request.object_name);

  return true;
}

bool TrayManagerServer::trayCapacityCB(thorp_msgs::TrayCapacityRequest& request,
                                       thorp_msgs::TrayCapacityResponse& response)
{
  response.total = slots_x_ * slots_y_;
  response.current = tray_full_ ? 0 : slots_x_ - next_x_ + ((slots_y_ - 1) - next_y_) * slots_x_;
  return true;
}

bool TrayManagerServer::trayNextPoseCB(thorp_msgs::TrayNextPoseRequest& request,
                                       thorp_msgs::TrayNextPoseResponse& response)
{
  if (tray_full_)
  {
    ROS_ERROR_NAMED(LOGNAME, "Tray is full");
    return false;
  }

  response.pose_on_tray = nextPose(offset_z_);
  ROS_INFO_NAMED(LOGNAME, "Next pose on tray at %s", ttk::toCStr3D(response.pose_on_tray));
  return true;
}

geometry_msgs::PoseStamped TrayManagerServer::nextPose(double z)
{
  ROS_ASSERT_MSG(!tray_full_, "Calling nextPose when the tray is already full");

  // Get next empty location coordinates
  double x = (next_x_ - slots_x_ / 2) * tray_slot_ + offset_x_;
  double y = (next_y_ - slots_y_ / 2) * tray_slot_ + offset_y_;
  next_x_ = (next_x_ + 1) % slots_x_;
  if (next_x_ == 0)
  {
    next_y_ = (next_y_ + 1) % slots_y_;
    if (next_x_ == next_y_)
    {
      tray_full_ = true;
    }
  }

  geometry_msgs::PoseStamped pose = ttk::createPose(x, y, z, 0, 0, 0, tray_link_);
  return pose;
}

void TrayManagerServer::visualizePlacePoses()
{
  geometry_msgs::PoseStamped ref_pose = ttk::createPose(0, 0, 0, tray_link_);
  std::vector<geometry_msgs::Point> points;
  for (int i = 0; i < slots_x_ * slots_y_; ++i)
  {
    points.push_back(nextPose(0.01).pose.position);
  }
  std::vector<std_msgs::ColorRGBA> colors(points.size(), ttk::namedColor("red"));

  visualization_.addMarkers({ ttk::Visualization::createPointList(ref_pose, points, colors, 0.01) });
  visualization_.publishMarkers();

  tray_full_ = false;  // calling nextPose for all slots will make the tray "full"!
}

}  // namespace thorp::manipulation
