/*
 * arduino.cpp
 *
 *  Created on: Apr 28, 2014
 *      Author: Jorge Santos
 */

#include <typeinfo>

#include "thorp_boards/arduino.hpp"

namespace thorp
{

  ArduinoNode::ArduinoNode(ros::NodeHandle& n)
             : wrong_readings(0), read_frequency(0.0), is_connected(false) {}

  ArduinoNode::~ArduinoNode() {}

  bool ArduinoNode::spin()
  {
    unsigned long long i = 0;
    ros::Rate rate(read_frequency);

    while (ros::ok())
    {
      if (is_connected)
      {
        if ((sonars.read() == false) || (irSensors.read() == false))
        {
          // Arduino returned 0 for a sonar or infrared sensor; normally this means we have reading
          // problems: the board is disconnected, or serial communication is not well synchronized
          if ((++wrong_readings % 10) == 9)
          {
            // Retry reinitializing Arduino interface
            ROS_WARN("Arduino interface: %d consecutive wrong readings; trying to reinitialize...",
                      wrong_readings);
            wrong_readings = 0;
            arduino_iface.reset(new ArduinoInterface(arduino_port));
            if (arduino_iface->initialize() == false)
            {
              ROS_ERROR("Arduino interface reinitialization failed on port %s. Stopping node...",
                         arduino_port.c_str());
              return false;
            }
          }
          continue;
        }
        wrong_readings = 0;
        sonars.trigger();
      }
      else {
        is_connected = connect();
      }
      ros::spinOnce();
      rate.sleep();
    }

    return true;
  }

  bool ArduinoNode::init()
  {
    ros::NodeHandle nh;
    std::string default_port("/dev/arduino");

    // General configuration
    nh.param("arduino_node/read_frequency", read_frequency, 20.0);
    nh.param("arduino_node/arduino_port", arduino_port, default_port);

    // Sonars configuration
    if (sonars.init("arduino_node/sonars", true) == false)
    {
      ROS_ERROR("Initialize sonars array failed");
      return false;
    }

    // IR sensors configuration
    if (irSensors.init("arduino_node/infrared") == false)
    {
      ROS_ERROR("Initialize IR sensors array failed");
      return false;
    }

    return true;
  }

  bool ArduinoNode::connect()
  {
    // Open an interface with the arduino board
    arduino_iface.reset(new ArduinoInterface(arduino_port));
    if (arduino_iface->initialize() == false)
    {
      ROS_ERROR("Arduino interface initialization failed on port %s", arduino_port.c_str());
      return false;
    }

    ROS_INFO("Arduino interface opened on port %s", arduino_iface->getID().c_str());

    if (sonars.connect(arduino_iface) == false)
    {
      ROS_ERROR("Connect sonar drivers failed");
      return false;
    }

    if (irSensors.connect(arduino_iface) == false)
    {
      ROS_ERROR("Connect IR sensor drivers failed");
      return false;
    }

    ROS_INFO("Arduino interface successfully initialized with %lu sonars and %lu IR sensors",
             sonars.size(), irSensors.size());
    return true;
  }

  bool ArduinoNode::validMap(const XmlRpc::XmlRpcValue& map, const std::string& name,
                             const XmlRpc::XmlRpcValue::Type type, int size)
  {
    // Validate map: it must be a list of 'size' elements of type 'type'
    if ((map.valid() == false) || (map.getType() != XmlRpc::XmlRpcValue::TypeArray))
    {
      ROS_ERROR("Invalid %s map: must be a list of %d %s", name.c_str(), size,
                type == XmlRpc::XmlRpcValue::TypeInt ? "integers" : "strings");
      return false;
    }
    if (map.size() != size)
    {
      ROS_ERROR("Invalid %s map: size doesn't match rangers count (%d != %d)",
                name.c_str(), map.size(), size);
      return false;
    }

    for (unsigned int i = 0; i < map.size(); i++)
    {
      if (map[i].getType() != type)
      {
        ROS_ERROR("Invalid %s map: element %d is not %s (%d != %d)", name.c_str(), i,
                   type == XmlRpc::XmlRpcValue::TypeInt ? "an integer" : "a string",
                   map[i].getType(), type);
        return false;
      }
    }

    return true;
  }


  ///////////////////////////////////////
  // Rangers list class implementation //
  ///////////////////////////////////////

  bool ArduinoNode::RangersList::init(const std::string& params_namespace, bool sonars)
  {
    this->sonars = sonars;

    ros::NodeHandle nh;

    // Rangers array parameters;
    // note that for all maps, the default value is an empty list, what is invalid
    int    rangers_count;
    double range_variance;
    double maximum_range;
    double minimum_range;
    double field_of_view;
    XmlRpc::XmlRpcValue input_pins_map;
    XmlRpc::XmlRpcValue ctrl_pins_map;
    XmlRpc::XmlRpcValue frame_ids_map;
    XmlRpc::XmlRpcValue topic_names_map;
    std::string         topic_namespace;

    nh.param(params_namespace + "/rangers_count",   rangers_count,  -1);  // mandatory
    nh.param(params_namespace + "/range_variance",  range_variance,  0.025);
    nh.param(params_namespace + "/maximum_range",   maximum_range,   6.5);
    nh.param(params_namespace + "/minimum_range",   minimum_range,   0.1);
    nh.param(params_namespace + "/field_of_view",   field_of_view,   0.1);
    nh.param(params_namespace + "/ctrl_pins_map",   ctrl_pins_map,   ctrl_pins_map);
    nh.param(params_namespace + "/input_pins_map",  input_pins_map,  input_pins_map);
    nh.param(params_namespace + "/frame_ids_map",   frame_ids_map,   frame_ids_map);
    nh.param(params_namespace + "/topic_names_map", topic_names_map, topic_names_map);
    nh.param(params_namespace + "/topic_namespace", topic_namespace, std::string());

    // Validate rangers count and maps: missconfiguration in one of them will
    // almost surely make something fail, so we abort if something is wrong
    if (rangers_count == -1)
    {
      ROS_ERROR("Missing mandatory parameter rangers count");
      return false;
    }

    if ((ArduinoNode::validMap(input_pins_map, params_namespace + "/input_pins_map",
                               XmlRpc::XmlRpcValue::TypeInt, rangers_count) == false) ||
        (ArduinoNode::validMap(frame_ids_map, params_namespace + "/frame_ids_map",
                               XmlRpc::XmlRpcValue::TypeString, rangers_count) == false) ||
        (ArduinoNode::validMap(topic_names_map, params_namespace + "/topic_names_map",
                               XmlRpc::XmlRpcValue::TypeString, rangers_count) == false))
    {
      return false;
    }

    if ((this->sonars == true) &&
        (ArduinoNode::validMap(ctrl_pins_map, params_namespace + "/ctrl_pins_map",
                               XmlRpc::XmlRpcValue::TypeInt, rangers_count) == false))
    {
      return false;
    }

    // Ready to initialize the ranger sensors list
    this->resize(rangers_count, Ranger());

    // Fill all fields on sensors array except the Bosch drivers, because
    // we must be connected to the Arduino before calling their constructor
    for (unsigned int i = 0; i < this->size(); i++)
    {
      (*this)[i].last_range = 0.0;
      (*this)[i].Q = 0.001;
      (*this)[i].R = range_variance; //0.0288;
      (*this)[i].P = (*this)[i].R;
      (*this)[i].input_pin = static_cast<int>(input_pins_map[i]);

      if (this->sonars == true)
        (*this)[i].ctrl_pin = static_cast<int>(ctrl_pins_map[i]);

      // Fill also all constant values on each range message
      (*this)[i].msg.header.frame_id = static_cast<std::string>(frame_ids_map[i]);
      (*this)[i].msg.radiation_type = (sonars == true) ?
                                       static_cast<uint8_t>(sensor_msgs::Range::ULTRASOUND)
                                     : static_cast<uint8_t>(sensor_msgs::Range::INFRARED);
      (*this)[i].msg.field_of_view = field_of_view;
      (*this)[i].msg.min_range = minimum_range;
      (*this)[i].msg.max_range = maximum_range;

      // We need individual publishers for each sensor
      std::string topic_name = topic_namespace + "/" + static_cast<std::string>(topic_names_map[i]);
      (*this)[i].pub = nh.advertise<sensor_msgs::Range>(topic_name, 1);

      char ctrl_pin_str[22];
      sonars ? sprintf(ctrl_pin_str, "control pin [DIO%d], ", (*this)[i].ctrl_pin)
             : sprintf(ctrl_pin_str, "");
      ROS_DEBUG("%s %d: input pin [A%d], %stopic [%s], frame [%s]", sonars?"Sonar":"IR sensor", i,
                (*this)[i].input_pin, ctrl_pin_str, (*this)[i].pub.getTopic().c_str(),
                (*this)[i].msg.header.frame_id.c_str());
    }

    return true;
  }

  bool ArduinoNode::RangersList::connect(const boost::shared_ptr<ArduinoInterface>& arduino_iface)
  {
    // Create an ADC driver (reading range) for every ranger
    for (unsigned int i = 0; i < this->size(); i++)
    {
      (*this)[i].adc_driver.reset(new AdcDriver(arduino_iface.get(), (*this)[i].input_pin));
      (*this)[i].adc_driver->setReference(5000); // can be also 1100 or 2560

      // Sonars also need a GPIO driver to trigger readings (Sharp sensors don't need
      // to be triggered; they just operate continuously as long as current is provided)
      if (this->sonars == true)
      {
        (*this)[i].gpio_driver.reset(new GpioDriver(arduino_iface.get(), (*this)[i].ctrl_pin));
      }
    }
    return true;
  }

  /**
   * Reads, linearizes, filters (KF) and publish the full array of range sensors.
   * @return False if we made a wrong reading (Arduino returned 0). True otherwise.
   */
  bool ArduinoNode::RangersList::read()
  {
    for (unsigned int i = 0; i < this->size(); i++)
    {
      // Read ranger i digitized voltage
      uint32_t reading = (*this)[i].adc_driver->read();

      if (reading == 0)  // we never read 0 in normal operation
        return false;

      double v, r;
      if (this->sonars == true)
      {
        // Convert voltage to range for sonars
        v = reading/1000000.0; // this is a magic number that works... but I cannot explain why!
        r = v*2.59183673469;   // that is, 25.4/9.8, as sonar doc claims that it reports ~9.8mV/in
      }
      else
      {
        // Linearize voltage and convert to range for IR sensors. The 6th grade polynomial's
        // coefficients where calculated by feeding real data to Matlab function polyfit
        v = reading/5000.0;    // I wonder if this is because the reference voltage is 5000...
        r = 1.06545479706866e-15*pow(v, 6) - 2.59219822235705e-12*pow(v, 5)
          + 2.52095247302813e-09*pow(v, 4) - 1.25091335895759e-06*pow(v, 3)
          + 0.000334991560873548*pow(v, 2) - 0.0469975280676629*v + 3.01895762047759;
      }

      // Update and publish range message
      (*this)[i].msg.header.seq = (*this)[i].msg.header.seq + 1;
      (*this)[i].msg.header.stamp = ros::Time::now();
      (*this)[i].msg.range = (*this)[i].updateFilter(r); /// TODO not bounding min/max ranges!!!  distanceKF <= (*this)[i].msg.max_range ? distanceKF : (*this)[i].infinity_range;

      (*this)[i].pub.publish((*this)[i].msg);
    }
    return true;
  }

  /**
   * Trigger one reading on all sonars. Has no effect on IR sensors
   * @return True if there aren't IO failures. False otherwise.
   */
  bool ArduinoNode::RangersList::trigger()
  {
    if (this->sonars == false)
      return true;

    for (unsigned int i = 0; i < this->size(); i++)
    {
      // Set sonar control pin (RX) to HIGH for at least 20μS to start one range reading
      if (! (*this)[i].gpio_driver->set(true))
      {
        ROS_WARN("Arduino interface: GPIO driver failed to set sonar %d control pin to HIGH", i);
        return false;
      }
      ros::Duration(0.0001); // 100 μs
      if (! (*this)[i].gpio_driver->set(false))
      {
        ROS_WARN("Arduino interface: GPIO driver failed to set sonar %d control pin to LOW", i);
        return false;
      }
    }
    return true;
  }


  /////////////////////////////////
  // Ranger class implementation //
  /////////////////////////////////

  double ArduinoNode::Ranger::updateFilter(double range)
  {
    // KF - estimate - prediction
    P = P + Q;

    // KF - correction
    double z = range - last_range;
    double Z = R + P;

    double K = P/Z;

    double rangeKF = last_range + K*z;
    P = P - K*P;

    // KF - collect data
    last_range = rangeKF;

    // Return filtered range
    return rangeKF;
  }
} // namespace thorp

