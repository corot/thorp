/*
 * Author: Jorge Santos
 */

#include <ros/exceptions.h>

#include "thorp_toolkit/kobuki_base.hpp"

namespace thorp::toolkit
{

std::string bumperName(uint8_t bumper)
{
  switch (bumper)
  {
    case kobuki_msgs::BumperEvent::LEFT:
      return "left";
    case kobuki_msgs::BumperEvent::CENTER:
      return "center";
    case kobuki_msgs::BumperEvent::RIGHT:
      return "right";
    default:
      throw ros::InvalidParameterException("Unknown bumper index " + std::to_string(bumper));
  }
}

std::string bumperAction(uint8_t action)
{
  switch (action)
  {
    case kobuki_msgs::BumperEvent::PRESSED:
      return "pressed";
    case kobuki_msgs::BumperEvent::RELEASED:
      return "released";
    default:
      throw ros::InvalidParameterException("Unknown bumper action " + std::to_string(action));
  }
}


}  // namespace thorp::toolkit
