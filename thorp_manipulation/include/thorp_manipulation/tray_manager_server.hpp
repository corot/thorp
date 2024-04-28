/*
 * Author: Jorge Santos
 */

#pragma once

#include <thorp_msgs/ClearPlanningScene.h>
#include <thorp_msgs/TrayAddObject.h>
#include <thorp_msgs/TrayCapacity.h>
#include <thorp_msgs/TrayNextPose.h>

#include <thorp_toolkit/planning_scene.hpp>
#include <thorp_toolkit/visualization.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::manipulation
{

/**
 * Server to manage the access to thorp's tray:
 *  - Provides total and current capacity
 *  - Provides the next pose where to place
 *  - Places objects on tray correctly on the planning scene
 *  - Allows clearing the planning scene but keeping the tray and its content
 */
class TrayManagerServer
{
public:
  TrayManagerServer();
  ~TrayManagerServer() = default;

private:
  static constexpr char LOGNAME[] = "tray_mng";

  std::vector<std::string> objs_on_tray_;

  ros::NodeHandle pnh_;
  ros::Publisher arm_trajectory_pub_;
  ros::ServiceServer tray_add_object_srv_;
  ros::ServiceServer tray_capacity_srv_;
  ros::ServiceServer tray_next_pose_srv_;
  ros::ServiceServer clear_pl_scene_srv_;
  ttk::PlanningScene& planning_scene_;
  ttk::Visualization visualization_;

  std::string tray_name_;
  std::string tray_link_;
  double tray_slot_ = 0.0;
  double tray_side_x_ = 0.0;
  double tray_side_y_ = 0.0;
  bool tray_full_ = false;
  int slots_x_ = 0;
  int slots_y_ = 0;
  double offset_x_ = 0.0;
  double offset_y_ = 0.0;
  double offset_z_ = 0.0;
  int next_x_ = 0;
  int next_y_ = 0;

  bool clearPlanningSceneCB(thorp_msgs::ClearPlanningSceneRequest& request,
                            thorp_msgs::ClearPlanningSceneResponse& response);

  /**
   * Move a collision object to the tray.
   * We don't check the capacity here, as we assume that the pose was obtained through TrayNextPose service.
   * @return Always true
   */
  bool trayAddObjectCB(thorp_msgs::TrayAddObjectRequest& request, thorp_msgs::TrayAddObjectResponse& response);

  /**
   * Request tray total and current capacity.
   * @return Always true
   */
  bool trayCapacityCB(thorp_msgs::TrayCapacityRequest& request, thorp_msgs::TrayCapacityResponse& response);

  /**
   * Get the next available pose on tray.
   * If this is the last available spot, we set tray full to true, so subsequent calls will fail.
   * @return False if the tray is already full, true otherwise
   */
  bool trayNextPoseCB(thorp_msgs::TrayNextPoseRequest& request, thorp_msgs::TrayNextPoseResponse& response);

  void visualizePlacePoses();

  geometry_msgs::PoseStamped nextPose(double z);
};

}  // namespace thorp::manipulation
