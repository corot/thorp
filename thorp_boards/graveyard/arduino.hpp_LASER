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
       * Inner class implementing the commons of any ranger (mostly the EKF filter)
       */
      class Ranger
      {
        public:
          double Q, R, P;    // KF values
          double last_range; /**< Internal field to store the latest reading */

          double updateFilter(double range);
      };

      /**
       * Inner class implementing an individual sonar
       */
      class Sonar : public Ranger
      {
        public:
    	  int ctrl_pin;
          int input_pin;
          boost::shared_ptr<AdcDriver> adc_driver;
          boost::shared_ptr<GpioDriver> gpio_driver;
      };

      /**
       * Inner class implementing an individual IR sensor
       */
      class IrSensor : public Ranger
      {
        public:
          int input_pin;
          ros::Publisher pub;
          sensor_msgs::Range msg;
          boost::shared_ptr<AdcDriver> adc_driver;
      };

      /*********************
      ** Initialization
      **********************/
      ArduinoNode(ros::NodeHandle& n);
      ~ArduinoNode();

      bool init();
      bool spin();

    protected:
      bool connect();
      bool readSonars();
      bool triggerSonars();
      bool readIrSensors();

    private:
      ros::NodeHandle nh;

      int    wrong_readings;
      double read_frequency;
      double sonar_ring_rad;
      std::string arduino_port;

//      int    us_rangers_count,  ir_rangers_count;
//      double us_range_variance, ir_range_variance;
//      double us_maximum_range,  ir_maximum_range;
//      double us_minimum_range,  ir_minimum_range;
//      double us_infinity_range, ir_infinity_range;
//      double us_ring_radius;
//      std::string us_frame_id;   /**< Frame id for the sonars' output laser scan */
//      XmlRpc::XmlRpcValue us_ctrl_pins_map, us_input_pins_map;
//      XmlRpc::XmlRpcValue ir_input_pins_map, ir_frame_ids_map;
//
//
//      int      wrong_readings;
//      int      rangers_count;
//      double   range_variance;
//      double   maximum_range;
//      double   infinity_range;
//      double   read_frequency;
//      double   ir_ring_radius;
//      std::string ir_frame_id;   /**< Frame id for the output laser scan */
//      std::string arduino_port;
//      XmlRpc::XmlRpcValue ctrl_pins_map;


      // Sonars stuff: message, publisher and objects array
      sensor_msgs::LaserScan scan;
      std::vector<Sonar> sonars;
      ros::Publisher sonars_pub;

      // IR sensors stuff: all contained within the objects array
      std::vector<IrSensor> irSensors;

      boost::shared_ptr<ArduinoInterface> arduino_iface;

      bool is_connected;

      bool validMap(const XmlRpc::XmlRpcValue& map,
                    const std::string& name, int size, XmlRpc::XmlRpcValue::Type type);
  };

} // namespace thorp

#endif /* ARDUINO_HPP_ */
