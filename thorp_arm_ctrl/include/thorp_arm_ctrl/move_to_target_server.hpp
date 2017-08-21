/*
 * Author: Jorge Santos
 */

#pragma once


// action servers
#include <actionlib/server/simple_action_server.h>
#include <thorp_msgs/MoveToTargetAction.h>

#include "thorp_arm_ctrl/thorp_arm_controller.hpp"


namespace thorp_arm_ctrl
{

/**
 * Action server providing basic arm motions:
 *  - move to a named target
 *  - move to a joint state configuration
 *  - move to a particular pose target
 */
class MoveToTargetServer : public ThorpArmController
{
private:
  actionlib::SimpleActionServer<thorp_msgs::MoveToTargetAction> as_;
  std::string action_name_;

public:
  MoveToTargetServer(const std::string name);
  ~MoveToTargetServer();

  void executeCB(const thorp_msgs::MoveToTargetGoal::ConstPtr& goal);
  void preemptCB();

private:
  /**
   * Move arm to a named configuration, normally described in the robot semantic description SRDF file.
   * @param target Named target to achieve
   * @return Thorp arm controller error code
   */
  int32_t moveArmTo(const std::string& target);

  /**
   * Move arm to a given configuration.
   * @param target joint state configuration to achieve
   * @return Thorp arm controller error code
   */
  int32_t moveArmTo(const sensor_msgs::JointState& target);

  /**
   * Move arm to a target pose. Only position coordinates are taken into account; the
   * orientation is calculated according to the direction and distance to the target.
   * @param target Pose target to achieve
   * @return Thorp arm controller error code
   */
  int32_t moveArmTo(const geometry_msgs::PoseStamped& target);
};

};
