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
#include <thorp_msgs/PlaceObjectAction.h>

#include "thorp_arm_ctrl/thorp_arm_controller.hpp"


namespace thorp_arm_ctrl
{

class PlaceObjectServer : public ThorpArmController
{
private:
  actionlib::SimpleActionServer<thorp_msgs::PlaceObjectAction> as_;
  std::string action_name_;

  thorp_msgs::PlaceObjectFeedback     feedback_;
  thorp_msgs::PlaceObjectResult       result_;
  thorp_msgs::PlaceObjectGoalConstPtr goal_;

  // Move groups to control arm and gripper with MoveIt!
//  moveit::planning_interface::MoveGroup arm_;
//  moveit::planning_interface::MoveGroup gripper_;
//
//  // We use the planning scene to gather information of tabletop/attached objects
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
//
//  // Pick and place parameters
//  std::string arm_link;
//  double gripper_open;
//  double attach_time;
//  double detach_time;
//  double z_backlash;

  const int PICK_ATTEMPTS = 5;
  const int PLACE_ATTEMPTS = PICK_ATTEMPTS;

public:
  PlaceObjectServer(const std::string name);
  ~PlaceObjectServer();

  void goalCB();
  void preemptCB();

private:
  bool place(const std::string& obj_name, const geometry_msgs::PoseStamped& pose);

};

};
