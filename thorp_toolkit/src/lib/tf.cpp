/*
 * Author: Jorge Santos
 */

#include <tf/transform_listener.h>

#include "thorp_toolkit/tf.hpp"


namespace thorp_toolkit
{


bool transformPose(const std::string& from_frame, const std::string& to_frame,
                   const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose)
{
  try
  {
    static tf::TransformListener tf_listener;
    tf_listener.waitForTransform(to_frame, from_frame, ros::Time(0.0), ros::Duration(1.0));
    tf_listener.transformPose(to_frame, in_pose, out_pose);
    return true;
  }
  catch (tf::InvalidArgument& e)
  {
    ROS_ERROR("Pose to transform has invalid orientation: %s", e.what());
    return false;
  }
  catch (tf::TransformException& e)
  {
    ROS_ERROR("Could not get '%s' to '%s' transform: %s", from_frame.c_str(), to_frame.c_str(), e.what());
    return false;
  }
}

bool transformPose(const std::string& from_frame, const std::string& to_frame,
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

char ___buffers___[10][256];
int ___next_buffer___ = -1;


std::string vector2str3D(const geometry_msgs::Vector3& vector)
{
  return vector2cstr3D(vector);
}

std::string vector2str3D(const geometry_msgs::Vector3Stamped& vector)
{
  return vector2cstr3D(vector.vector);
}

std::string point2str2D(const geometry_msgs::Point& point)
{
  return point2cstr2D(point);
}

std::string point2str2D(const geometry_msgs::PointStamped& point)
{
  return point2cstr2D(point.point);
}

std::string point2str3D(const geometry_msgs::Point& point)
{
  return point2cstr3D(point);
}

std::string point2str3D(const geometry_msgs::PointStamped& point)
{
  return point2cstr3D(point.point);
}

std::string pose2str2D(const geometry_msgs::Pose& pose)
{
  return pose2cstr2D(pose);
}

std::string pose2str2D(const geometry_msgs::PoseStamped& pose)
{
  return pose2cstr2D(pose.pose);
}

std::string pose2str2D(const tf::Stamped<tf::Pose>& pose)
{
  return pose2cstr2D(pose);
}

std::string pose2str3D(const geometry_msgs::Pose& pose)
{
  return pose2cstr3D(pose);
}

std::string pose2str3D(const geometry_msgs::PoseStamped& pose)
{
  return pose2cstr3D(pose.pose);
}

std::string pose2str3D(const tf::Stamped<tf::Pose>& pose)
{
  return pose2cstr3D(pose);
}


const char* vector2cstr3D(const geometry_msgs::Vector3& vector)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f", vector.x, vector.y, vector.z);
  return ___buffers___[___next_buffer___];
}

const char* vector2cstr3D(const geometry_msgs::Vector3Stamped& vector)
{
  return vector2cstr3D(vector.vector);
}

const char* point2cstr2D(const geometry_msgs::Point& point)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f", point.x, point.y);
  return ___buffers___[___next_buffer___];
}

const char* point2cstr2D(const geometry_msgs::PointStamped& point)
{
  return point2cstr2D(point.point);
}

const char* point2cstr3D(const geometry_msgs::Point& point)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f", point.x, point.y, point.z);
  return ___buffers___[___next_buffer___];
}

const char* point2cstr3D(const geometry_msgs::PointStamped& point)
{
  return point2cstr3D(point.point);
}

const char* pose2cstr2D(const geometry_msgs::Pose& pose)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f", pose.position.x, pose.position.y, yaw(pose));
  return ___buffers___[___next_buffer___];
}

const char* pose2cstr2D(const geometry_msgs::PoseStamped& pose)
{
  return pose2cstr2D(pose.pose);
}

const char* pose2cstr2D(const tf::Stamped<tf::Pose>& pose)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f", pose.getOrigin().x(), pose.getOrigin().y(), yaw(pose));
  return ___buffers___[___next_buffer___];
}

const char* pose2cstr3D(const geometry_msgs::Pose& pose)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f,  %.2f, %.2f, %.2f",
          pose.position.x, pose.position.y, pose.position.z, roll(pose), pitch(pose), yaw(pose));
  return ___buffers___[___next_buffer___];
}

const char* pose2cstr3D(const geometry_msgs::PoseStamped& pose)
{
  return pose2cstr3D(pose.pose);
}

const char* pose2cstr3D(const tf::Stamped<tf::Pose>& pose)
{
  ++___next_buffer___; ___next_buffer___ %= 10;
  sprintf(___buffers___[___next_buffer___], "%.2f, %.2f, %.2f,  %.2f, %.2f, %.2f",
          pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(), roll(pose), pitch(pose), yaw(pose));
  return ___buffers___[___next_buffer___];
}


} /* namespace thorp_toolkit */
