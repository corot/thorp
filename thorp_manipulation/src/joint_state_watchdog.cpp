/*
 * Author: Jorge Santos
 */

#include "thorp_manipulation/joint_state_watchdog.hpp"


namespace thorp::manipulation
{

JointStateWatchdog::JointStateWatchdog()
{
  ros::NodeHandle nh;
  gripper_pad_width = nh.param<double>("gripper_controller/pad_width", 0.01);
  gripper_finger_length = nh.param("gripper_controller/finger_length", 0.02);
  gripper_min_opening = nh.param("gripper_controller/min_opening", 0.0);
  gripper_max_opening = nh.param("gripper_controller/max_opening", 0.09);
  gripper_center = nh.param("gripper_controller/center", 0.0);
  gripper_invert = nh.param("gripper_controller/invert", false);
  gripper_joint = nh.param("gripper_controller/joint", std::string("gripper_joint"));

  gripper_opening.resize(HISTORY_SIZE, NAN);
  gripper_effort.resize(HISTORY_SIZE, NAN);

  // subscribe to joint_states topic
  joint_state_sub = nh.subscribe("joint_states", 100, &JointStateWatchdog::jointStateCB, this);

  // and republish filtering-out all non-arm joint messages
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("arm_joint_states", 100);
}

JointStateWatchdog::~JointStateWatchdog()
{
}


void JointStateWatchdog::jointStateCB(const sensor_msgs::JointState& msg)
{
  // round     std::trunc(100.0 * cmd_vel.angular.z) / 100.0

  if (gripper_joint_index < 0)
  {
    auto it = std::find(msg.name.begin(), msg.name.end(), gripper_joint);
    if (it == msg.name.end())
      return;
    gripper_joint_index = it - msg.name.begin();
  }
  else if (msg.name.size() <= gripper_joint_index || msg.name[gripper_joint_index] != gripper_joint)
    return;

  // invert the gripper controller one-side-gripper model to get opening out of joint position
  double theta = ((gripper_invert ? -1 : +1) * msg.position[gripper_joint_index]) + gripper_center;

  gripper_opening.push_back(2.0 * gripper_finger_length * sin(theta) + gripper_pad_width);
  gripper_opening.pop_front();
  gripper_effort.push_back(msg.effort[gripper_joint_index]);
  gripper_effort.pop_front();

  joint_state_pub.publish(msg);
  // ROS_ERROR_THROTTLE(1, "%f  %f    %d", gripper_opening.back(), gripper_effort.back(), gripper_joint_index);
}

}  // namespace thorp::manipulation
