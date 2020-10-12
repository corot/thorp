/*
 * Author: Jorge Santos
 */

#include "thorp_toolkit/geometry.hpp"


namespace thorp_toolkit
{


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
