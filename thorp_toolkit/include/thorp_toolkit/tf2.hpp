/*
 * Author: Jorge Santos
 */

#pragma once

#include <mutex>

#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


namespace thorp::toolkit
{

/**
 * Singleton encapsulating a tf2 stuff
 */
class TF2
{
private:
  static std::unique_ptr<TF2> inst_ptr_;
  static std::mutex mutex_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster bcaster_;
  tf2_ros::StaticTransformBroadcaster sbcaster_;

  TF2() : listener_(buffer_) {}

public:
  ~TF2() = default;

  /**
   * Singletons should not be cloneable.
   */
  TF2(TF2 &other) = delete;

  /**
   * Singletons should not be assignable.
   */
  void operator=(const TF2 &) = delete;

  /**
   * This is the static method that controls the access to the singleton instance.
   * On the first run, it creates a singleton object and places it into the static
   * field. On subsequent runs, it returns a reference to the existing object.
   */
  static TF2& instance();

  /**
   * Provide access to the TF2 buffer.
   * @return Reference to TF2 buffer.
   */
  tf2_ros::Buffer& buffer();

  /**
   * Get the transform between two frames by frame ID.
   * @param target_frame The frame to which data should be transformed
   * @param source_frame The frame where the data originated
   * @param time The time at which the value of the transform is desired. (0 will get the latest)
   * @param timeout How long to block before failing. Defaults to 2.0, enough time to fill an empty
   * buffer, so we can call this function anytime without calling instance beforehand.
   * @return true if transformation succeeded.
   */
  bool lookupTransform(const std::string& target_frame, const std::string& source_frame,
                       geometry_msgs::TransformStamped& transform,
                       const ros::Time& time = ros::Time(), const ros::Duration& timeout = ros::Duration(2));

  /**
   * Transform in_pose to the target frame
   * @param target_frame target frame
   * @param in_pose initial pose
   * @param out_pose transformed pose
   * @param timeout How long to block before failing. Defaults to 2.0, enough time to fill an empty
   * buffer, so we can call this function anytime without calling instance beforehand.
   * @return true if transformation succeeded.
   */
  bool transformPose(const std::string& target_frame,
                     const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose,
                     const ros::Duration& timeout = ros::Duration(2));

  /**
   * Get the transform between two frames by frame ID.
   * @param target_frame The frame to which data should be transformed
   * @param source_frame The frame where the data originated
   * @param in_pose initial pose
   * @param out_pose transformed pose
   * @param time The time at which the value of the transform is desired. (0 will get the latest)
   * @param timeout How long to block before failing. Defaults to 2.0, enough time to fill an empty
   * buffer, so we can call this function anytime without calling instance beforehand.
   * @return true if transformation succeeded.
   */
  bool transformPose(const std::string& target_frame, const std::string& source_frame,
                     const geometry_msgs::Pose& in_pose, geometry_msgs::Pose& out_pose,
                     const ros::Time& time = ros::Time(), const ros::Duration& timeout = ros::Duration(2));

  void sendTransform(const geometry_msgs::TransformStamped& tf);

  void sendTransform(const geometry_msgs::Transform& tf, const std::string& from, const std::string& to);

  void sendTransform(const geometry_msgs::Pose& pose, const std::string& from, const std::string& to);
};

/**
 * Convert a transform into a pose msg
 * @param tf input
 * @param pose output
 */
void tf2pose(const tf::Transform& tf, geometry_msgs::Pose& pose);

/**
 * Convert a transform msg into a pose msg
 * @param tf input
 * @param pose output
 */
void tf2pose(const geometry_msgs::Transform& tf, geometry_msgs::Pose& pose);

/**
 * Convert a stamped transform into a stamped pose msg
 * @param tf input
 * @param pose output
 */
void tf2pose(const tf::StampedTransform& tf, geometry_msgs::PoseStamped& pose);

/**
 * Convert a stamped transform msg into a stamped pose msg
 * @param tf input
 * @param pose output
 */
 void tf2pose(const geometry_msgs::TransformStamped& tf, geometry_msgs::PoseStamped& pose);

/**
 * Convert a pose msg into a transform
 * @param pose input
 * @param tf output
 */
void pose2tf(const geometry_msgs::Pose& pose, tf::Transform& tf);

/**
 * Convert a stamped pose msg into a stamped transform
 * @param pose input
 * @param tf output
 */
 void pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf);


} /* namespace thorp::toolkit */
