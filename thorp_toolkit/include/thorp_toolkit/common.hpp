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
std_msgs::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f);

/**
 * @brief Make a random RGBA color msg
 * @param seed randomizer seed
 * @param alpha transparency
 * @return RGBA color msg
 */
std_msgs::ColorRGBA randomColor(unsigned int seed = 0, float alpha = 1.0f);

/**
 * @brief Make a RGBA color msg for a color name
 * @param color_name color by name
 * @param alpha transparency
 * @return RGBA color msg
 * @throw  std::out_of_range  If color_name doesn't exist
 */
std_msgs::ColorRGBA namedColor(const std::string& color_name, float alpha = 1.0f);

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
