/*
 * arduino_node.cpp
 *
 *  Created on: Apr 28, 2014
 *      Author: Jorge Santos
 */

#include "thorp_boards/arduino.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arduino");

  ros::NodeHandle nh;

  thorp::ArduinoNode node(nh);

  if (node.init() == true)
  {
    ROS_INFO("Arduino interface node : initialized");
    node.spin();
    return 0;
  }

  ROS_ERROR("Arduino interface node : initialization failed. Aborting...");
  return -1;
}
