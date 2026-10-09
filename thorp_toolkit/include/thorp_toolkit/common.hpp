/*
 * Author: Jorge Santos
 */

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace thorp::toolkit
{

/**
 * @brief QoS of a latched topic: subscribers joining later still get its last message
 */
inline const rclcpp::QoS LATCHED = rclcpp::QoS(1).transient_local();

/**
 * @brief Initialize the toolkit with the node its singletons and functions use to access ROS.
 * Call once, before using anything else that needs a node.
 * @param node the application's node
 */
void init(const rclcpp::Node::SharedPtr& node);

/**
 * @brief The node given to init
 * @return node shared pointer
 * @throw  std::runtime_error  If init hasn't been called
 */
rclcpp::Node::SharedPtr node();

/**
 * @brief Logger for the toolkit functions
 * @return the toolkit logger
 */
rclcpp::Logger logger();

/**
 * @brief Make a RGBA color msg
 * @param r red
 * @param g green
 * @param b blue
 * @param a alpha
 * @return RGBA color msg
 */
std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f);

/**
 * @brief Make a random RGBA color msg
 * @param seed randomizer seed
 * @param alpha transparency
 * @return RGBA color msg
 */
std_msgs::msg::ColorRGBA randomColor(unsigned int seed = 0, float alpha = 1.0f);

/**
 * @brief Make a RGBA color msg for a color name
 * @param color_name color by name
 * @param alpha transparency
 * @return RGBA color msg
 * @throw  std::out_of_range  If color_name doesn't exist
 */
std_msgs::msg::ColorRGBA namedColor(const std::string& color_name, float alpha = 1.0f);

/**
 * @brief Name the color of a CIELAB value, as one of the names namedColor knows
 * @param lightness L*
 * @param a a*, green to red
 * @param b b*, blue to yellow
 * @return color name, e.g. "red" or "light gray"
 */
std::string colorName(float lightness, float a, float b);

/**
 * @brief Split a comma-separated string into a vector of strings
 * @param csv Comma-separated values string
 * @return elements as a vector of strings
 */
std::vector<std::string> tokenize(const std::string& csv);

/**
 * @brief Receive one message from a topic.
 * This will create a new subscription to the topic, receive one message, then unsubscribe.
 * @param topic name of topic
 * @param timeout timeout; negative (the default) waits forever
 * @return Optional message of type MessageType; std::nullopt on timeout or ROS shutdown
 */
template <typename MessageType>
std::optional<MessageType> waitForMessage(const std::string& topic,
                                          std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1))
{
  MessageType received_message;
  if (rclcpp::wait_for_message(received_message, node(), topic, timeout))
  {
    return received_message;
  }
  return std::nullopt;
}

} /* namespace thorp::toolkit */
