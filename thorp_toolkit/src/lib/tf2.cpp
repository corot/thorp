/*
 * Author: Jorge Santos
 */
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "thorp_toolkit/tf2.hpp"


namespace thorp_toolkit
{

// Static attributes
tf2_ros::Buffer TF2::buffer_;
std::unique_ptr<TF2> TF2::inst_ptr_;
std::mutex TF2::mutex_;

/**
 * The first time we call getInstance we will lock the storage location and create the single instance.
 * It must be called on every static method.
 */
TF2& TF2::getInstance()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (inst_ptr_ == nullptr)
  {
    inst_ptr_ = std::unique_ptr<TF2>(new TF2);
  }
  return *inst_ptr_;
}

bool TF2::transformPose(const std::string& from_frame, const std::string& to_frame,
                        const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose)
{
  try
  {
    TF2::getInstance();  // so we start listening if not done before
    buffer_.transform(in_pose, out_pose, to_frame, ros::Duration(2.0));  // 2.0 is enough to fill enough the buffer
    // transform returns a reference; and no idea if it throws any exception;  TODO remake this
    return true;
  }
  catch (tf::InvalidArgument& e)
  {
    ROS_ERROR("Invalid input pose: %s", e.what());
    return false;
  }
  catch (tf::TransformException& e)
  {
    ROS_ERROR("Could not get '%s' to '%s' transform: %s", from_frame.c_str(), to_frame.c_str(), e.what());
    return false;
  }
}

bool TF2::transformPose(const std::string& from_frame, const std::string& to_frame,
                        const geometry_msgs::Pose& in_pose, geometry_msgs::Pose& out_pose)
{
  geometry_msgs::PoseStamped in_stamped;
  geometry_msgs::PoseStamped out_stamped;

  in_stamped.header.frame_id = from_frame;
  in_stamped.pose = in_pose;
  if (transformPose(from_frame, to_frame, in_stamped, out_stamped))
  {
    out_pose = out_stamped.pose;
    return true;
  }

  return false;
}


void tf2pose(const tf::Transform& tf, geometry_msgs::Pose& pose)
{
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  tf::quaternionTFToMsg(tf.getRotation(), pose.orientation);
}

void tf2pose(const tf::StampedTransform& tf, geometry_msgs::PoseStamped& pose)
{
  pose.header.stamp    = tf.stamp_;
  pose.header.frame_id = tf.frame_id_;
  tf2pose(tf, pose.pose);
}

void pose2tf(const geometry_msgs::Pose& pose, tf::Transform& tf)
{
  tf.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation, q);
  tf.setRotation(q);
}

void pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf)
{
  tf.stamp_    = pose.header.stamp;
  tf.frame_id_ = pose.header.frame_id;
  pose2tf(pose.pose, tf);
}


} /* namespace thorp_toolkit */
