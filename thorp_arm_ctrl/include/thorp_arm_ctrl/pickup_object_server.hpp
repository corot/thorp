/*
 * Author: Jorge Santos
 */

#pragma once


// Thorp pickup object action server
#include <actionlib/server/simple_action_server.h>
#include <thorp_msgs/PickupObjectAction.h>

// MoveIt! pickup action client
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/PickupAction.h>

#include "thorp_arm_ctrl/thorp_arm_controller.hpp"


namespace thorp_arm_ctrl
{

class PickupObjectServer : public ThorpArmController
{
private:
  // Thorp pickup object action server
  actionlib::SimpleActionServer<thorp_msgs::PickupObjectAction> as_;
  std::string action_name_;

  // MoveIt! pickup action client
  actionlib::SimpleActionClient<moveit_msgs::PickupAction> ac_;

  const int PICK_ATTEMPTS = 5;

public:
  PickupObjectServer(const std::string name);
  ~PickupObjectServer();

  void executeCB(const thorp_msgs::PickupObjectGoal::ConstPtr& goal);
  void preemptCB();

private:
  int32_t pickup(const std::string& obj_name, const std::string& surface, float max_effort = 0.0);

  int32_t makeGrasps(const geometry_msgs::PoseStamped& target_pose, const geometry_msgs::Vector3& target_size,
                     const std::string& target_name, const std::string& surface, float max_effort,
                     std::vector<moveit_msgs::Grasp>& grasps);
};

};
