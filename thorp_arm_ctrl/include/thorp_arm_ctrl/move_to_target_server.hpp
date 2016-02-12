/*
 * Copyright (c) 2015, Jorge Santos
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Vanadium Labs LLC nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Jorge Santos
 */

#pragma once

#include <ros/ros.h>

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

  thorp_msgs::MoveToTargetFeedback     feedback_;
  thorp_msgs::MoveToTargetResult       result_;
  thorp_msgs::MoveToTargetGoalConstPtr goal_;

//  ros::Publisher target_pose_pub_;
//
//  // Move groups to control arm and gripper with MoveIt!
//  moveit::planning_interface::MoveGroup arm_;
//  moveit::planning_interface::MoveGroup gripper_;

public:
  MoveToTargetServer(const std::string name);
  ~MoveToTargetServer();

  void goalCB();
  void preemptCB();

private:
  /**
   * Move arm to a named configuration, normally described in the robot semantic description SRDF file.
   * @param target Named target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const std::string& target);

  /**
   * Move arm to a target pose. Only position coordinates are taken into account; the
   * orientation is calculated according to the direction and distance to the target.
   * @param target Pose target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const geometry_msgs::PoseStamped& target);

  /**
   * Set gripper opening.
   * @param opening Physical opening of the gripper, in meters
   * @param wait_for_complete Wait or not for the execution of the trajectory to complete
   * @return True of success, false otherwise
   */
  bool setGripper(float opening, bool wait_for_complete = true);
};

};
