/*
 * Author: Jorge Santos
 */

#pragma once

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo_grasp_plugin_ros/GazeboGraspEvent.h>

#include "thorp_manipulation/thorp_arm_controller.hpp"
#include "thorp_manipulation/joint_state_watchdog.hpp"


namespace thorp_manipulation
{

/**
 * Server providing some house-keeping and recovery actions:
 *  - escape from a self-colliding configuration
 *  - check if the gripper is holding an object (both physically and according to the planning scene)
 *  - clear any object attached to the gripper
 */
class HouseKeepingServer : public ThorpArmController
{
private:
  ros::Publisher     arm_trajectory_pub_;
  ros::ServiceClient planning_scene_srv_;
  ros::ServiceServer force_resting_srv_;
  ros::ServiceServer clear_gripper_srv_;
  ros::ServiceServer obj_attached_srv_;
  ros::ServiceServer gripper_busy_srv_;
  ros::Subscriber    grasp_events_sub_;
  gazebo_grasp_plugin_ros::GazeboGraspEvent last_grasp_event_;

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
   * Check if we have a collision object attached to the gripper, according to the planning scene.
   * @return True if succeeded, false otherwise
   */
  bool objAttachedCB(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response);

  /**
   * Check if the gripper is physically holding an object, regardless of what the planning scene says.
   * @return True if succeeded, false otherwise
   */
  bool gripperBusyCB(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response);

  void graspEventCB(const gazebo_grasp_plugin_ros::GazeboGraspEvent& event);
};

};
