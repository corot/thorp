/*
 * Author: Jorge Santos
 */

#pragma once

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "thorp_arm_ctrl/thorp_arm_controller.hpp"
#include "thorp_arm_ctrl/joint_state_watchdog.hpp"


namespace thorp_arm_ctrl
{

/**
 * Server providing some recovery actions:
 *  - escape from a self-colliding configuration
 *  - clear any object attached to the gripper
 */
class HouseKeepingServer : public ThorpArmController
{
private:
  ros::Publisher     arm_trajectory_pub_;
  ros::ServiceClient planning_scene_srv_;
  ros::ServiceServer force_resting_srv_;
  ros::ServiceServer clear_gripper_srv_;
  ros::ServiceServer gripper_busy_srv_;

  moveit_msgs::PlanningScene planning_scene_;

  JointStateWatchdog joint_state_watchdog_;

public:
  HouseKeepingServer();
  ~HouseKeepingServer();

private:

  /**
   * Move the arm to resting pose regardless of shelf-collisions.
   * @return True if succeeded, false otherwise
   */
  bool forceRestingCB(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response);

  /**
   * Remove any attached object and open the gripper to let it fall.
   * @return True if succeeded, false otherwise
   */
  bool clearGripperCB(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response);

  /**
   * Check if the gripper is holding an object.
   * @return True if succeeded, false otherwise
   */
  bool gripperBusyCB(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response);
};

};
