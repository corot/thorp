/*
 * Author: Jorge Santos
 */

#pragma once

#include <memory>
#include <mutex>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace thorp::toolkit
{

/**
 * Singleton encapsulating a tf2 stuff, using the node given to thorp::toolkit::init
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

  TF2();

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
   * Destroy the singleton instance, if any; the next call to instance will create a new one.
   * thorp::toolkit::init makes the context call it on shutdown.
   */
  static void destroy();

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
                       geometry_msgs::msg::TransformStamped& transform,
                       const tf2::TimePoint& time = tf2::TimePointZero,
                       const tf2::Duration& timeout = tf2::durationFromSec(2.0));

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
                     const geometry_msgs::msg::PoseStamped& in_pose, geometry_msgs::msg::PoseStamped& out_pose,
                     const tf2::Duration& timeout = tf2::durationFromSec(2.0));

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
                     const geometry_msgs::msg::Pose& in_pose, geometry_msgs::msg::Pose& out_pose,
                     const tf2::TimePoint& time = tf2::TimePointZero,
                     const tf2::Duration& timeout = tf2::durationFromSec(2.0));

  void sendTransform(const geometry_msgs::msg::TransformStamped& tf);

  void sendTransform(const geometry_msgs::msg::Transform& tf, const std::string& from, const std::string& to);

  void sendTransform(const geometry_msgs::msg::Pose& pose, const std::string& from, const std::string& to);
};

/**
 * Convert a transform into a pose msg
 * @param tf input
 * @param pose output
 */
void tf2pose(const tf2::Transform& tf, geometry_msgs::msg::Pose& pose);

/**
 * Convert a transform msg into a pose msg
 * @param tf input
 * @param pose output
 */
void tf2pose(const geometry_msgs::msg::Transform& tf, geometry_msgs::msg::Pose& pose);

/**
 * Convert a stamped transform into a stamped pose msg
 * @param tf input
 * @param pose output
 */
void tf2pose(const tf2::Stamped<tf2::Transform>& tf, geometry_msgs::msg::PoseStamped& pose);

/**
 * Convert a stamped transform msg into a stamped pose msg
 * @param tf input
 * @param pose output
 */
void tf2pose(const geometry_msgs::msg::TransformStamped& tf, geometry_msgs::msg::PoseStamped& pose);

/**
 * Convert a pose msg into a transform
 * @param pose input
 * @param tf output
 */
void pose2tf(const geometry_msgs::msg::Pose& pose, tf2::Transform& tf);

/**
 * Convert a pose msg into a transform msg
 * @param pose input
 * @param tf output
 */
void pose2tf(const geometry_msgs::msg::Pose& pose, geometry_msgs::msg::Transform& tf);


} /* namespace thorp::toolkit */
