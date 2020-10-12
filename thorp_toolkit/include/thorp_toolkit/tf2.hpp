/*
 * Author: Jorge Santos
 */

#pragma once

#include <mutex>

#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


namespace thorp_toolkit
{

/**
 * Singleton encapsulating a tf2 stuff
 */
class TF2
{
private:
  static std::unique_ptr<TF2> inst_ptr_;
  static std::mutex mutex_;

  static tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

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
  static TF2& getInstance();

  /**
   * Transform in_pose from on frame to another
   * @param from_frame initial frame
   * @param to_frame target frame
   * @param in_pose initial pose
   * @param out_pose transformed pose
   * @return true if transformation succeeded.
   */
  static bool transformPose(const std::string& from_frame, const std::string& to_frame,
                            const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose);

  /**
   * Transform in_pose from on frame to another
   * @param from_frame initial frame
   * @param to_frame target frame
   * @param in_pose initial pose
   * @param out_pose transformed pose
   * @return true if transformation succeeded.
   */
  static bool transformPose(const std::string& from_frame, const std::string& to_frame,
                            const geometry_msgs::Pose& in_pose, geometry_msgs::Pose& out_pose);
};


void tf2pose(const tf::Transform& tf, geometry_msgs::Pose& pose);
void tf2pose(const tf::StampedTransform& tf, geometry_msgs::PoseStamped& pose);

void pose2tf(const geometry_msgs::Pose& pose, tf::Transform& tf);
void pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf);


} /* namespace thorp_toolkit */
