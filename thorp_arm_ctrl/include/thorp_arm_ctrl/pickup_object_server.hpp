/*
 * Author: Jorge Santos
 */

#pragma once


// action servers
#include <actionlib/server/simple_action_server.h>
#include <thorp_msgs/PickupObjectAction.h>

#include "thorp_arm_ctrl/thorp_arm_controller.hpp"


namespace thorp_arm_ctrl
{

class PickupObjectServer : public ThorpArmController
{
private:
  actionlib::SimpleActionServer<thorp_msgs::PickupObjectAction> as_;
  std::string action_name_;

  const int PICK_ATTEMPTS = 5;

public:
  PickupObjectServer(const std::string name);
  ~PickupObjectServer();

  void goalCB();
  void preemptCB();

private:
  int32_t pickup(const std::string& obj_name, const std::string& surface);
};

};
