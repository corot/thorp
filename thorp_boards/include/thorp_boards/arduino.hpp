/*
 * arduino.hpp
 *
 *  Created on: Apr 28, 2014
 *      Author: Jorge Santos
 */

#ifndef ARDUINO_HPP_
#define ARDUINO_HPP_


#include <ros/ros.h>
#include <arduino_interface.hpp>
#include <adc_driver/adc_driver.h>
#include <gpio_driver/gpio_driver.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>

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
          boost::shared_ptr<AdcDriver> adc_driver;   /**< Driver required to read an analog input */
          boost::shared_ptr<GpioDriver> gpio_driver; /**< Driver required to write to a digital output */

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
          bool connect(const boost::shared_ptr<ArduinoInterface>& arduino_iface);

          bool read();
          bool trigger();
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
      int    wrong_readings;
      double read_frequency;
      std::string arduino_port;

      RangersList sonars;
      RangersList irSensors;

      boost::shared_ptr<ArduinoInterface> arduino_iface;

      bool is_connected;

      static bool validMap(const XmlRpc::XmlRpcValue& map, const std::string& name,
                           const XmlRpc::XmlRpcValue::Type type, int size);
  };

} // namespace thorp

#endif /* ARDUINO_HPP_ */
