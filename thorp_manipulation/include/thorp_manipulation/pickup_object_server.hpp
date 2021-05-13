/*
 * Author: Jorge Santos
 */

#pragma once

// Thorp pickup object action server
#include <actionlib/server/simple_action_server.h>
#include <thorp_msgs/PickupObjectAction.h>

#include "thorp_manipulation/thorp_arm_controller.hpp"


namespace thorp_manipulation
{

class PickupObjectServer : public ThorpArmController
{
private:
  // Thorp pickup object action server
  actionlib::SimpleActionServer<thorp_msgs::PickupObjectAction> as_;

  const int PICK_ATTEMPTS = 5;

public:
  explicit PickupObjectServer(const std::string& name);
  ~PickupObjectServer();

  void executeCB(const thorp_msgs::PickupObjectGoal::ConstPtr& goal);
  void preemptCB();

private:
  int32_t pickup(const std::string& obj_name, const std::string& surface,
                 const float max_effort = 0.0, const float tightening = 0.001);

  int32_t makeGrasps(const geometry_msgs::PoseStamped& obj_pose, const geometry_msgs::Vector3& obj_size,
                     const std::string& target_name, const std::string& surface,
                     const float max_effort, const float tightening,
                     std::vector<moveit_msgs::Grasp>& grasps);

  double gripperClosing(const geometry_msgs::PoseStamped& grasp_pose,
                        const geometry_msgs::PoseStamped& obj_pose,
                        const geometry_msgs::Vector3& obj_size,
                        const float gripper_tightening_closing);
};

};
