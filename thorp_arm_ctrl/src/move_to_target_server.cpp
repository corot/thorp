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

#include "thorp_arm_ctrl/move_to_target_server.hpp"


namespace thorp_arm_ctrl
{


MoveToTargetServer::MoveToTargetServer(const std::string name) :
  as_(name, false), action_name_(name)
{
  // Register the goal and feedback callbacks
  as_.registerGoalCallback(boost::bind(&MoveToTargetServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&MoveToTargetServer::preemptCB, this));
  as_.start();
}

MoveToTargetServer::~MoveToTargetServer()
{
  as_.shutdown();
}

void MoveToTargetServer::goalCB()
{
  ROS_INFO("[move to target] Received goal!");
  goal_ = as_.acceptNewGoal();
  bool result = false;

  switch (goal_->target_type)
  {
    case thorp_msgs::MoveToTargetGoal::NAMED_TARGET:
      result = moveArmTo(goal_->named_target);
      break;
    case thorp_msgs::MoveToTargetGoal::JOINT_STATE:
    case thorp_msgs::MoveToTargetGoal::POSE_TARGET:
    default:
      ROS_ERROR("[move to target] Move to target of type %d not implemented", goal_->target_type);
      break;
  }

  if (result)
  {
    as_.setSucceeded(result_);
  }
  else
  {
    as_.setAborted(result_);
  }
}

void MoveToTargetServer::preemptCB()
{
  ROS_INFO("[move to target] %s: Preempted", action_name_.c_str());
  gripper_.stop();
  arm_.stop();

  // set the action state to preempted
  as_.setPreempted();
}

bool MoveToTargetServer::moveArmTo(const std::string& target)
{
  if (target == "resting")  // XXX temporal cheat... blocks less fov to the camera
    setGripper(0.002, false);

  ROS_DEBUG("[move to target] Move arm to '%s' position", target.c_str());
  if (arm_.setNamedTarget(target) == false)
  {
    ROS_ERROR("[move to target] Set named target '%s' failed", target.c_str());
    return false;
  }

  moveit::planning_interface::MoveItErrorCode result = arm_.move();
  if (bool(result) == true)
  {
    ROS_INFO("[move to target] Move to target \"%s\" completed", target.c_str());
    return true;
  }
  else
  {
    ROS_ERROR("[move to target] Move to target \"%s\" failed (error %d)", target.c_str(), result.val);
    return false;
  }
}


bool MoveToTargetServer::moveArmTo(const geometry_msgs::PoseStamped& target)
{
  int attempts = 0;
  ROS_DEBUG("[move to target] Move arm to [%.2f, %.2f, %.2f, %.2f]",
           target.pose.position.x, target.pose.position.y, target.pose.position.z,
           tf::getYaw(target.pose.orientation));
  while (attempts < 5)
  {
    geometry_msgs::PoseStamped modiff_target = target;

    double x = modiff_target.pose.position.x;
    double y = modiff_target.pose.position.y;
    double z = modiff_target.pose.position.z;
    double d = sqrt(x*x + y*y);
    if (d > 0.3)
    {
      // Maximum reachable distance by the turtlebot arm is 30 cm
      ROS_ERROR("[move to target] Target pose out of reach [%f > %f]", d, 0.3);
      return false;
    }
    // Pitch is 90 (vertical) at 10 cm from the arm base; the farther the target is, the closer to horizontal
    // we point the gripper. Yaw is the direction to the target. We also try some random variations of both to
    // increase the chances of successful planning.
    double rp = M_PI_2 - std::asin((d - 0.1)/0.205); // 0.205 = arm's max reach - vertical pitch distance + ε
    double ry = std::atan2(y, x);

    tf::Quaternion q = tf::createQuaternionFromRPY(0.0,
                                                   attempts*fRand(-0.05, +0.05) + rp,
                                                   attempts*fRand(-0.05, +0.05) + ry);
    tf::quaternionTFToMsg(q, modiff_target.pose.orientation);

    // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner
    ROS_DEBUG("[move to target] Z increase:  %f  +  %f", modiff_target.pose.position.z, std::abs(std::cos(rp))/50.0);
    modiff_target.pose.position.z += std::abs(std::cos(rp))/50.0;

    ROS_DEBUG("[move to target] Set pose target [%.2f, %.2f, %.2f] [d: %.2f, p: %.2f, y: %.2f]", x, y, z, d, rp, ry);
    target_pose_pub_.publish(modiff_target);

    if (arm_.setPoseTarget(modiff_target) == false)
    {
      ROS_ERROR("[move to target] Set pose target [%.2f, %.2f, %.2f, %.2f] failed",
                modiff_target.pose.position.x, modiff_target.pose.position.y, modiff_target.pose.position.z,
                tf::getYaw(modiff_target.pose.orientation));
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result = arm_.move();
    if (bool(result) == true)
    {
      ROS_INFO("[move to target] Move to target [%.2f, %.2f, %.2f, %.2f] completed",
               modiff_target.pose.position.x, modiff_target.pose.position.y, modiff_target.pose.position.z,
               tf::getYaw(modiff_target.pose.orientation));
      return true;
    }
    else
    {
      ROS_ERROR("[move to target] Move to target failed (error %d) at attempt %d",
                result.val, attempts + 1);
    }
    attempts++;
  }

  ROS_ERROR("[move to target] Move to target failed after %d attempts", attempts);
  return false;
}

bool MoveToTargetServer::setGripper(float opening, bool wait_for_complete)
{
  ROS_DEBUG("[move to target] Set gripper opening to %f", opening);
  if (gripper_.setJointValueTarget("gripper_joint", opening) == false)
  {
    ROS_ERROR("[move to target] Set gripper opening to %f failed", opening);
    return false;
  }

  moveit::planning_interface::MoveItErrorCode result =
      wait_for_complete ? gripper_.move() : gripper_.asyncMove();
  if (result == true)
  {
    return true;
  }
  else
  {
    ROS_ERROR("[move to target] Set gripper opening failed (error %d)", result.val);
    return false;
  }
}


};
