/*
 * Author: Jorge Santos
 */

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "thorp_toolkit/tf2.hpp"


namespace thorp_toolkit
{

void tf2pose(const tf::Transform& tf, geometry_msgs::Pose& pose)
{
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  tf::quaternionTFToMsg(tf.getRotation(), pose.orientation);
}

void tf2pose(const geometry_msgs::Transform& tf, geometry_msgs::Pose& pose)
{
  pose.position.x = tf.translation.x;
  pose.position.y = tf.translation.y;
  pose.position.z = tf.translation.z;
  pose.orientation = tf.rotation;
}

void tf2pose(const tf::StampedTransform& tf, geometry_msgs::PoseStamped& pose)
{
  pose.header.stamp    = tf.stamp_;
  pose.header.frame_id = tf.frame_id_;
  tf2pose(tf, pose.pose);
}

void tf2pose(const geometry_msgs::TransformStamped& tf, geometry_msgs::PoseStamped& pose)
{
  pose.header.stamp    = tf.header.stamp;
  pose.header.frame_id = tf.header.frame_id;
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = tf.transform.translation.z;
  pose.pose.orientation = tf.transform.rotation;
}

void pose2tf(const geometry_msgs::Pose& pose, tf::Transform& tf)
{
  tf.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation, q);
  tf.setRotation(q);
}

void pose2tf(const geometry_msgs::Pose& pose, geometry_msgs::Transform& tf)
{
  tf.translation.x = pose.position.x;
  tf.translation.y = pose.position.y;
  tf.translation.z = pose.position.z;
  tf.rotation = pose.orientation;
}


// TF2 class implementation

// Static attributes
std::unique_ptr<TF2> TF2::inst_ptr_;
std::mutex TF2::mutex_;

/**
 * The first time we call instance we will lock the storage location and create the single instance.
 * It must be called on every static method.
 */
TF2& TF2::instance()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (inst_ptr_ == nullptr)
  {
    inst_ptr_ = std::unique_ptr<TF2>(new TF2);
  }
  return *inst_ptr_;
}

tf2_ros::Buffer& TF2::buffer()
{
  return buffer_;
}

bool TF2::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                          geometry_msgs::TransformStamped& transform,
                          const ros::Time& time, const ros::Duration& timeout)
{
  try
  {
    transform = buffer_.lookupTransform(target_frame, source_frame, time, timeout);
    return true;
  }
  catch (tf2::LookupException& e)
  {
    ROS_ERROR("Lookup error: %s", e.what());
  }
  catch (tf2::ConnectivityException& e)
  {
    ROS_ERROR("Unconnected frames: %s", e.what());
  }
  catch (tf2::ExtrapolationException& e)
  {
    ROS_ERROR("Extrapolation error: %s", e.what());
  }
  catch (tf2::InvalidArgumentException& e)
  {
    ROS_ERROR("Invalid input pose: %s", e.what());
  }
  return false;
}

bool TF2::transformPose(const std::string& target_frame, const geometry_msgs::PoseStamped& in_pose,
                        geometry_msgs::PoseStamped& out_pose, const ros::Duration& timeout)
{
  try
  {
    buffer_.transform(in_pose, out_pose, target_frame, timeout);
    return true;
  }
  catch (tf2::LookupException& e)
  {
    ROS_ERROR("Lookup error: %s", e.what());
  }
  catch (tf2::ConnectivityException& e)
  {
    ROS_ERROR("Unconnected frames: %s", e.what());
  }
  catch (tf2::ExtrapolationException& e)
  {
    ROS_ERROR("Extrapolation error: %s", e.what());
  }
  catch (tf2::InvalidArgumentException& e)
  {
    ROS_ERROR("Invalid input pose: %s", e.what());
  }
  return false;
}

bool TF2::transformPose(const std::string& target_frame, const std::string& source_frame,
                        const geometry_msgs::Pose& in_pose, geometry_msgs::Pose& out_pose,
                        const ros::Time& time, const ros::Duration& timeout)
{
  geometry_msgs::PoseStamped in_stamped;
  geometry_msgs::PoseStamped out_stamped;

  in_stamped.header.frame_id = source_frame;
  in_stamped.header.stamp = time;
  in_stamped.pose = in_pose;
  if (transformPose(target_frame, in_stamped, out_stamped, timeout))
  {
    out_pose = out_stamped.pose;
    return true;
  }

  return false;
}

void TF2::sendTransform(const geometry_msgs::TransformStamped& tf)
{
  sbcaster_.sendTransform(tf);
}

void TF2::sendTransform(const geometry_msgs::Transform& tf, const std::string& from, const std::string& to)
{
  geometry_msgs::TransformStamped tfs;
  tfs.header.stamp = ros::Time::now();
  tfs.header.frame_id = from;
  tfs.child_frame_id = to;
  tfs.transform = tf;
  sbcaster_.sendTransform(tfs);
}

void TF2::sendTransform(const geometry_msgs::Pose& pose, const std::string& from, const std::string& to)
{
  geometry_msgs::TransformStamped tfs;
  tfs.header.stamp = ros::Time::now();
  tfs.header.frame_id = from;
  tfs.child_frame_id = to;
  pose2tf(pose, tfs.transform);
  sbcaster_.sendTransform(tfs);
}

} /* namespace thorp_toolkit */
