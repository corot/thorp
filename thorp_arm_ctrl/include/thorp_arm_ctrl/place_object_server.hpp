/*
 * Author: Jorge Santos
 */

#pragma once


// action servers
#include <actionlib/server/simple_action_server.h>
#include <thorp_msgs/PlaceObjectAction.h>

#include "thorp_arm_ctrl/thorp_arm_controller.hpp"


namespace thorp_arm_ctrl
{

class PlaceObjectServer : public ThorpArmController
{
private:
  actionlib::SimpleActionServer<thorp_msgs::PlaceObjectAction> as_;
  std::string action_name_;

  const int PLACE_ATTEMPTS = 5;

public:
  PlaceObjectServer(const std::string name);
  ~PlaceObjectServer();

  void goalCB();
  void preemptCB();

private:
  int32_t place(const std::string& obj_name, const std::string& surface, const geometry_msgs::PoseStamped& pose);

};

};
