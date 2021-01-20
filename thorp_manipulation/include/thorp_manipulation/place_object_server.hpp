/*
 * Author: Jorge Santos
 */

#pragma once

// Thorp pickup object action server
#include <actionlib/server/simple_action_server.h>
#include <thorp_msgs/PlaceObjectAction.h>

#include "thorp_manipulation/thorp_arm_controller.hpp"


namespace thorp_manipulation
{

class PlaceObjectServer : public ThorpArmController
{
private:
  // Thorp pickup object action server
  actionlib::SimpleActionServer<thorp_msgs::PlaceObjectAction> as_;

  const int PLACE_ATTEMPTS = 5;

public:
  explicit PlaceObjectServer(const std::string& name);
  ~PlaceObjectServer();

  void executeCB(const thorp_msgs::PlaceObjectGoal::ConstPtr& goal);
  void preemptCB();

private:
  int32_t place(const std::string& obj_name, const std::string& surface, const geometry_msgs::PoseStamped& pose);

  int32_t makePlaceLocations(const geometry_msgs::PoseStamped& target_pose,
                             const geometry_msgs::PoseStamped& obj_pose,
                             const geometry_msgs::Vector3& obj_size,
                             const std::string& obj_name, const std::string& surface,
                             std::vector<moveit_msgs::PlaceLocation>& place_locations);
};

};
