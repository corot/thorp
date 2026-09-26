/*
 * Author: Jorge Santos
 */

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "thorp_toolkit/common.hpp"
#include "thorp_toolkit/tf2.hpp"

namespace thorp::toolkit
{

void tf2pose(const tf2::Transform& tf, geometry_msgs::msg::Pose& pose)
{
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  pose.orientation = tf2::toMsg(tf.getRotation());
}

void tf2pose(const geometry_msgs::msg::Transform& tf, geometry_msgs::msg::Pose& pose)
{
  pose.position.x = tf.translation.x;
  pose.position.y = tf.translation.y;
  pose.position.z = tf.translation.z;
  pose.orientation = tf.rotation;
}

void tf2pose(const tf2::Stamped<tf2::Transform>& tf, geometry_msgs::msg::PoseStamped& pose)
{
  pose.header.stamp = tf2_ros::toMsg(tf.stamp_);
  pose.header.frame_id = tf.frame_id_;
  tf2pose(tf, pose.pose);
}

void tf2pose(const geometry_msgs::msg::TransformStamped& tf, geometry_msgs::msg::PoseStamped& pose)
{
  pose.header.stamp = tf.header.stamp;
  pose.header.frame_id = tf.header.frame_id;
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = tf.transform.translation.z;
  pose.pose.orientation = tf.transform.rotation;
}

void pose2tf(const geometry_msgs::msg::Pose& pose, tf2::Transform& tf)
{
  tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  tf.setRotation(q);
}

void pose2tf(const geometry_msgs::msg::Pose& pose, geometry_msgs::msg::Transform& tf)
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

TF2::TF2()
  : buffer_(node()->get_clock())
  , listener_(buffer_, node())
  , bcaster_(node())
  , sbcaster_(node())
{
}

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

void TF2::destroy()
{
  std::lock_guard<std::mutex> lock(mutex_);
  inst_ptr_.reset();
}

tf2_ros::Buffer& TF2::buffer()
{
  return buffer_;
}

bool TF2::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                          geometry_msgs::msg::TransformStamped& transform, const tf2::TimePoint& time,
                          const tf2::Duration& timeout)
{
  try
  {
    transform = buffer_.lookupTransform(target_frame, source_frame, time, timeout);
    return true;
  }
  catch (tf2::LookupException& e)
  {
    RCLCPP_ERROR(logger(), "Lookup error: %s", e.what());
  }
  catch (tf2::ConnectivityException& e)
  {
    RCLCPP_ERROR(logger(), "Unconnected frames: %s", e.what());
  }
  catch (tf2::ExtrapolationException& e)
  {
    RCLCPP_ERROR(logger(), "Extrapolation error: %s", e.what());
  }
  catch (tf2::InvalidArgumentException& e)
  {
    RCLCPP_ERROR(logger(), "Invalid input pose: %s", e.what());
  }
  return false;
}

bool TF2::transformPose(const std::string& target_frame, const geometry_msgs::msg::PoseStamped& in_pose,
                        geometry_msgs::msg::PoseStamped& out_pose, const tf2::Duration& timeout)
{
  try
  {
    buffer_.transform(in_pose, out_pose, target_frame, timeout);
    return true;
  }
  catch (tf2::LookupException& e)
  {
    RCLCPP_ERROR(logger(), "Lookup error: %s", e.what());
  }
  catch (tf2::ConnectivityException& e)
  {
    RCLCPP_ERROR(logger(), "Unconnected frames: %s", e.what());
  }
  catch (tf2::ExtrapolationException& e)
  {
    RCLCPP_ERROR(logger(), "Extrapolation error: %s", e.what());
  }
  catch (tf2::InvalidArgumentException& e)
  {
    RCLCPP_ERROR(logger(), "Invalid input pose: %s", e.what());
  }
  return false;
}

bool TF2::transformPose(const std::string& target_frame, const std::string& source_frame,
                        const geometry_msgs::msg::Pose& in_pose, geometry_msgs::msg::Pose& out_pose,
                        const tf2::TimePoint& time, const tf2::Duration& timeout)
{
  geometry_msgs::msg::PoseStamped in_stamped;
  geometry_msgs::msg::PoseStamped out_stamped;
  in_stamped.header.frame_id = source_frame;
  in_stamped.header.stamp = tf2_ros::toMsg(time);
  in_stamped.pose = in_pose;
  if (transformPose(target_frame, in_stamped, out_stamped, timeout))
  {
    out_pose = out_stamped.pose;
    return true;
  }
  return false;
}

void TF2::sendTransform(const geometry_msgs::msg::TransformStamped& tf)
{
  sbcaster_.sendTransform(tf);
}

void TF2::sendTransform(const geometry_msgs::msg::Transform& tf, const std::string& from, const std::string& to)
{
  geometry_msgs::msg::TransformStamped tfs;
  tfs.header.stamp = node()->now();
  tfs.header.frame_id = from;
  tfs.child_frame_id = to;
  tfs.transform = tf;
  sbcaster_.sendTransform(tfs);
}

void TF2::sendTransform(const geometry_msgs::msg::Pose& pose, const std::string& from, const std::string& to)
{
  geometry_msgs::msg::TransformStamped tfs;
  tfs.header.stamp = node()->now();
  tfs.header.frame_id = from;
  tfs.child_frame_id = to;
  pose2tf(pose, tfs.transform);
  sbcaster_.sendTransform(tfs);
}

}  // namespace thorp::toolkit
