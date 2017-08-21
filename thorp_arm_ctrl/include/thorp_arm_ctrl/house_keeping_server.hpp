/*
 * Author: Jorge Santos
 */

#pragma once

#include <std_srvs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "thorp_arm_ctrl/thorp_arm_controller.hpp"


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

  moveit_msgs::PlanningScene planning_scene_;

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
};

};
