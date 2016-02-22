/*
 * arduino.hpp
 *
 *  Created on: Apr 28, 2014
 *      Author: Jorge Santos
 */

#pragma once


#include <ros/ros.h>
#include <serial/serial.h>

#include <sensor_msgs/Range.h>

namespace thorp
{

class ArduinoNode
{
public:
  /**
   * Inner class containing the necessary attributes to read from range sensors connected
   * to an Arduino board and publish the result as sensor_msgs/Range messages. Only a basic
   * KF filter is implemented.
   */
  class Ranger
  {
    public:
      double Q, R, P;    // KF values
      double last_range; /**< Internal field to store the latest reading */

      int ctrl_pin;  /**< Digital control pin to trigger ranger readings */
      int input_pin; /**< Analog input pin to which the ranger is connected */
      ros::Publisher pub;
      sensor_msgs::Range msg;

      /**
       * @brief  Filter a range and update filter internal fields
       * @param  range raw range reading
       * @return Filtered range
       */
      double updateFilter(double range);
  };

  /**
   * Inner class implementing a list of rangers
   */
  class RangersList : public std::vector<Ranger>
  {
    public:
      bool sonars;

      bool init(const std::string& params_namespace, bool sonars = false);
      bool read(uint16_t* readings);

    private:
      template <typename T>
      bool getParam(const std::string& key, T& value)
      {
        if (ros::NodeHandle().getParam(key, value) == false)
        {
          ROS_ERROR("Missing mandatory parameter %s", key.c_str());
          return false;
        }
        return true;
      }

      template <typename T>
      bool getParam(const std::string& key, std::vector<T>& value, int size)
      {
        if (ros::NodeHandle().getParam(key, value) == false)
        {
          ROS_ERROR("Mandatory parameter %s must be a list of %s",
                    key.c_str(), typeid(T) == typeid(int) ? "integers" : "strings");
          return false;
        }
        if (value.size() != size)
        {
          ROS_ERROR("Invalid %s map: size doesn't match rangers count (%lu != %d)",
                    key.c_str(), value.size(), size);
          return false;
        }
        return true;
      }
  };


  /*********************
  ** Initialization
  **********************/
  ArduinoNode(ros::NodeHandle& n);
  ~ArduinoNode();

  bool init();
  bool spin();
  bool connect();

private:
  double      read_frequency;
  unsigned int  read_timeout;
  std::string   arduino_port;
  serial::Serial serial_port;

  RangersList sonars;
  RangersList irSensors;
};


} // namespace thorp
