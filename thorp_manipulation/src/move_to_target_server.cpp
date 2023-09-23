/*
 * Author: Jorge Santos
 */

#include <tf/tf.h>

// auxiliary libraries
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;

#include "thorp_manipulation/move_to_target_server.hpp"


namespace thorp_manipulation
{


MoveToTargetServer::MoveToTargetServer(const std::string& name) :
    action_name_(name), as_(name, boost::bind(&MoveToTargetServer::executeCB, this, _1), false)
{
  // Register feedback callback for our server; executeCB is run on a separated thread, so it can be cancelled
  as_.registerPreemptCallback(boost::bind(&MoveToTargetServer::preemptCB, this));
  as_.start();
  ROS_INFO("[move to target] Server started");
}

MoveToTargetServer::~MoveToTargetServer()
{
  as_.shutdown();
}

void MoveToTargetServer::executeCB(const thorp_msgs::MoveToTargetGoal::ConstPtr& goal)
{
  preempted_ = false;
  thorp_msgs::MoveToTargetResult result;

  switch (goal->target_type)
  {
    case thorp_msgs::MoveToTargetGoal::NAMED_TARGET:
      result.error.code = moveArmTo(goal->named_target);
      break;
    case thorp_msgs::MoveToTargetGoal::JOINT_STATE:
      result.error.code = moveArmTo(goal->joint_state);
      break;
    case thorp_msgs::MoveToTargetGoal::POSE_TARGET:
      result.error.code = moveArmTo(goal->pose_target);
      break;
    default:
      result.error.code = thorp_msgs::ThorpError::INVALID_TARGET_TYPE;
      ROS_ERROR("[move to target] Move to target of type %d not implemented", goal->target_type);
      break;
  }

  result.error.text = mec2str(result.error);
  if (result.error.code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    as_.setSucceeded(result);
  }
  else if (result.error.code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    as_.setPreempted();
  }
  else
  {
    as_.setAborted(result);
  }
}

void MoveToTargetServer::preemptCB()
{
  ROS_WARN("[move to target] Action preempted; cancel all movement");
  preempted_ = true;
  gripper().stop();
  arm().stop();
}

int32_t MoveToTargetServer::moveArmTo(const std::string& target)
{
  ROS_DEBUG("[move to target] Move arm to '%s' position", target.c_str());
  if (!arm().setNamedTarget(target))
  {
    ROS_ERROR("[move to target] Set named target '%s' failed", target.c_str());
    return thorp_msgs::ThorpError::INVALID_NAMED_TARGET;
  }

  moveit::core::MoveItErrorCode result = arm().move();
  if (result)
  {
    ROS_INFO("[move to target] Move to target '%s' completed", target.c_str());
  }
  else if (preempted_)
  {
    ROS_WARN("[move to target] Move to target '%s' preempted", target.c_str());
    result.val = moveit::core::MoveItErrorCode::PREEMPTED;
  }
  else
  {
    ROS_ERROR("[move to target] Move to target '%s' failed: %s", target.c_str(), mec2str(result));
  }

  return result.val;
}

int32_t MoveToTargetServer::moveArmTo(const sensor_msgs::JointState& target)
{
  ROS_DEBUG_STREAM("[move to target] Move arm to target configuration:\n" << target);
  if (!arm().setJointValueTarget(target))
  {
    ROS_ERROR_STREAM("[move to target] Set joint value target failed:\n" << target);
    return thorp_msgs::ThorpError::INVALID_JOINT_STATE;
  }

  moveit::core::MoveItErrorCode result = arm().move();
  if (result)
  {
    ROS_INFO("[move to target] Move to joint value target completed");
  }
  else if (preempted_)
  {
    ROS_WARN("[move to target] Move to joint value target preempted");
    result.val = moveit::core::MoveItErrorCode::PREEMPTED;
  }
  else
  {
    ROS_ERROR("[move to target] Move to joint value target failed: %s", mec2str(result));
  }

  return result.val;
}

int32_t MoveToTargetServer::moveArmTo(const geometry_msgs::PoseStamped& target)
{
  ROS_DEBUG("[move to target] Move arm to [%s]", ttk::pose2cstr3D(target.pose));

  geometry_msgs::PoseStamped modiff_target = target;
  if (!validateTargetPose(modiff_target, false, false, false))
  {
    ROS_ERROR("[move to target] Invalid target pose [%s]", ttk::pose2cstr3D(target.pose));
    return thorp_msgs::ThorpError::INVALID_TARGET_POSE;
  }

  if (!arm().setPoseTarget(modiff_target))
  {
    ROS_ERROR("[move to target] Set pose target [%s] failed", ttk::pose2cstr3D(modiff_target.pose));
    return thorp_msgs::ThorpError::INVALID_TARGET_POSE;
  }

  moveit::core::MoveItErrorCode result = arm().move();
  if (result)
  {
    ROS_INFO("[move to target] Move to target [%s] completed", ttk::pose2cstr3D(modiff_target.pose));
  }
  else if (preempted_)
  {
    ROS_WARN("[move to target] Move to target [%s] preempted", ttk::pose2cstr3D(modiff_target.pose));
    result.val = moveit::core::MoveItErrorCode::PREEMPTED;
  }
  else
  {
    ROS_ERROR("[move to target] Move to target [%s] failed: %s", ttk::pose2cstr3D(modiff_target.pose), mec2str(result));
  }

  return result.val;
}

};
