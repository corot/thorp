/*
 * Author: Jorge Santos
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

namespace thorp_toolkit
{

/**
 * @brief Make a RGBA color msg
 * @param r red
 * @param g green
 * @param b blue
 * @param a alpha
 * @return RGBA color msg
 */
inline std_msgs::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

/**
 * @brief Make a random RGBA color msg
 * @param seed randomizer seed
 * @param alpha transparency
 * @return RGBA color msg
 */
inline std_msgs::ColorRGBA randomColor(unsigned int seed = 0, float alpha = 1.0f)
{
  srand(seed);
  std_msgs::ColorRGBA color;
  color.r = float(rand()) / RAND_MAX;
  color.g = float(rand()) / RAND_MAX;
  color.b = float(rand()) / RAND_MAX;
  color.a = alpha;
  return color;
}

/**
 * @brief Make a RGBA color msg for a color name
 * @param color_name color by name
 * @param alpha transparency
 * @return RGBA color msg
 * @throw  std::out_of_range  If color_name doesn't exist
 */
inline std_msgs::ColorRGBA namedColor(const std::string& color_name, float alpha = 1.0f)
{
  static std::map<std::string, std_msgs::ColorRGBA> color_map;
  if (color_map.empty())
  {
    color_map["red"].r = 1.0f;
    color_map["blue"].b = 1.0f;
    color_map["green"].g = 1.0f;
    color_map["yellow"].r = color_map["yellow"].g = 1.0f;
    color_map["orange"].r = 1.0f;
    color_map["orange"].g = 0.65f;
    color_map["white"].r = color_map["white"].g = color_map["white"].b = 1.0f;
    color_map["gray"].r = color_map["gray"].g = color_map["gray"].b = 0.5f;
    color_map["beige"].r = color_map["beige"].g = 0.96f;
    color_map["beige"].b = 0.86f;
  }
  std_msgs::ColorRGBA& color = color_map.at(color_name);
  color.a = alpha;
  return color;
}

/**
 * @brief Receive one message from a topic.
 * This will create a new subscription to the topic, receive one message, then unsubscribe.
 * @param topic name of topic
 * @param timeout timeout as ROS Duration (defaults to wait forever)
 * @return Optional message of type MessageType; std::nullopt on timeout or ROS shutdown
 */
template <typename MessageType>
std::optional<MessageType> waitForMessage(const std::string& topic, ros::Duration timeout = ros::Duration::ZERO)
{
  std::optional<MessageType> received_message;
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<MessageType>(topic, 1,
                                                  [&](auto msg)
                                                  {
                                                    received_message = *msg;
                                                    sub.shutdown();
                                                  });
  // Spin and wait for the message up to timeout
  ros::Time time_start = ros::Time::now();
  ros::Rate loop_rate(100);
  while (ros::ok() && !received_message && (timeout.isZero() || (ros::Time::now() - time_start) < timeout))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return received_message;
}

} /* namespace thorp_toolkit */
