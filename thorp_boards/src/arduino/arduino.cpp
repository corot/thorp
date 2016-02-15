/*
 * arduino.cpp
 *
 *  Created on: Apr 28, 2014
 *      Author: Jorge Santos
 */

#include "thorp_boards/arduino.hpp"

namespace thorp
{

  ArduinoNode::ArduinoNode(ros::NodeHandle& n)
             : read_frequency(0.0) {}

  ArduinoNode::~ArduinoNode() {}

  bool ArduinoNode::spin()
  {
    ros::Rate rate(read_frequency);

    // Arduino send us all readings packed as consecutive 2 bytes raw values; the first two bytes
    // signal the beginning of the buffer, so we can use them to deal with lost synchronization
    uint8_t num_readings = (sonars.size() + irSensors.size());
    uint8_t storage_buffer[(num_readings + 1) * sizeof(uint16_t)];

    while (ros::ok())
    {
      if (serial_port.isOpen())
      {
        try
        {
          if (serial_port.available() < sizeof(storage_buffer))
          {
            // We are reading data faster than the Arduino provides it; not a big
            // drama, we just wait a little bit so more bytes are received and retry
            ROS_DEBUG( "Not enough bytes available on serial port (%lu < %lu)",
                               serial_port.available(), sizeof(storage_buffer));
            ros::Duration(0.005).sleep();
          }
          else
          {
            // Enough data available to fill our buffer; keep moving

            if (serial_port.available() > sizeof(storage_buffer))
            {
              // We are reading data slower than the Arduino provides it; we are probably
              // using outdated data, so we should increase this node's reading rate
              ROS_WARN( "Too much bytes available on serial port (%lu > %lu)",
                                serial_port.available(), sizeof(storage_buffer));
              // XXX we could consume the excess of bytes, but it is probably better to
              // tweak frequencies instead, as a more stable and efficient solution
            }

            size_t bytes_read = serial_port.read(storage_buffer, sizeof(storage_buffer));

            // TODO: compare if bytes_read < sizeof(storage_buffer); means read timeout!
            uint16_t* readings = reinterpret_cast<uint16_t*>(&storage_buffer);
            if (readings[0] == 0xFFFF)
            {
              // Synchronization code 0xFFFF found; correct reading. Feed it to our sensors
              sonars.read(++readings);
              irSensors.read(readings + sonars.size());
            }
            else
            {
              // Reading out of synchronization; consume available bytes to make next reading valid
              uint8_t dump_buffer[serial_port.available()];
              ROS_WARN("Serial synchronization lost; reading %lu bytes to clear pending data (read: %lu)",
                       sizeof(dump_buffer), serial_port.read(dump_buffer, sizeof(dump_buffer)));
            }
          }
        }
        catch (serial::PortNotOpenedException &e)
        {
          ROS_ERROR("Arduino serial %s; trying to reconnect...", e.what());
          connect();
        }
        catch (serial::SerialException &e)
        {
          // Serial read failed; maybe the board is temporally disconnected...
          // keep trying to reconnect every half second, as kobuki base does
          ROS_ERROR("Arduino serial %s; trying to reconnect...", e.what());
          serial_port.close();
        }
        catch (serial::IOException &e)
        {
          // Serial read failed; maybe the board is temporally disconnected...
          // keep trying to reconnect every half second, as kobuki base does
          ROS_ERROR("Arduino serial %s; trying to reconnect...", e.what());
          serial_port.close();
        }
      }
      else {
        connect();

        // Wait a bit until first data gets available or before the next connection retry
        ros::Duration(0.5).sleep();
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

    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serial_port.setPort(arduino_port);
    serial_port.setBaudrate(115200);
    serial_port.setTimeout(timeout);

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
    try
    {
      // (Re)connect to Arduino serial port
      serial_port.open();
      ROS_INFO("Arduino connected on port %s; reading %lu sonars and %lu IR sensors",
               arduino_port.c_str(), sonars.size(), irSensors.size());
      return true;
    }
    catch (std::exception &e)
    {
      ROS_ERROR("Unable to connect with Arduino on port %s; %s", arduino_port.c_str(), e.what());
      return false;
    }
  }


  ///////////////////////////////////////
  // Rangers list class implementation //
  ///////////////////////////////////////

  bool ArduinoNode::RangersList::init(const std::string& params_namespace, bool sonars)
  {
    this->sonars = sonars;

    ros::NodeHandle nh;

    // Rangers array parameters;
    // note that all parameters except ctrl_pins_map and topic_namespace are mandatory
    // Actually, input and control pins maps are only to provide debug information and to
    // validate against the hardcoded values in the firmware, the ones that really matter
    int                      rangers_count;
    double                   range_variance;
    double                   maximum_range;
    double                   minimum_range;
    double                   field_of_view;
    std::vector<int>         input_pins_map;
    std::vector<int>         ctrl_pins_map;
    std::vector<std::string> frame_ids_map;
    std::vector<std::string> topic_names_map;
    std::string              topic_namespace;

    // We must thoroughly validate rangers count and maps: missconfiguration in one of
    // them will almost surely make something fail, so we abort if something is wrong
    bool allOK = true;
    allOK &= this->getParam(params_namespace + "/rangers_count",   rangers_count);
    allOK &= this->getParam(params_namespace + "/range_variance",  range_variance);
    allOK &= this->getParam(params_namespace + "/maximum_range",   maximum_range);
    allOK &= this->getParam(params_namespace + "/minimum_range",   minimum_range);
    allOK &= this->getParam(params_namespace + "/field_of_view",   field_of_view);
    allOK &= this->getParam(params_namespace + "/input_pins_map",  input_pins_map,  rangers_count);
    allOK &= this->getParam(params_namespace + "/frame_ids_map",   frame_ids_map,   rangers_count);
    allOK &= this->getParam(params_namespace + "/topic_names_map", topic_names_map, rangers_count);

    if (sonars)
    {
      // ctrl_pins_map is only mandatory for sonars
      allOK &= this->getParam(params_namespace + "/ctrl_pins_map", ctrl_pins_map,   rangers_count);
    }

    // Optional parameters
    if (nh.getParam(params_namespace + "/topic_namespace", topic_namespace) == false)
    {
      ROS_WARN("Topic namespace not specified; we set is as empty");
    }

    if (! allOK)
      return false;

    // Ready to initialize the ranger sensors list
    this->resize(rangers_count, Ranger());

    // Fill all constant fields on sensors array
    for (unsigned int i = 0; i < this->size(); i++)
    {
      (*this)[i].last_range = 0.0;
      (*this)[i].Q = 0.001;
      (*this)[i].R = range_variance; //0.0288;
      (*this)[i].P = (*this)[i].R;
      (*this)[i].input_pin = input_pins_map[i];

      if (this->sonars == true)
        (*this)[i].ctrl_pin = ctrl_pins_map[i];

      // Fill also all constant values on each range message
      (*this)[i].msg.header.frame_id = frame_ids_map[i];
      (*this)[i].msg.radiation_type = (sonars == true) ?
                                       static_cast<uint8_t>(sensor_msgs::Range::ULTRASOUND)
                                     : static_cast<uint8_t>(sensor_msgs::Range::INFRARED);
      (*this)[i].msg.field_of_view = field_of_view;
      (*this)[i].msg.min_range = minimum_range;
      (*this)[i].msg.max_range = maximum_range;

      // We need individual publishers for each sensor
      std::string topic_name = topic_namespace + "/" + topic_names_map[i];
      (*this)[i].pub = nh.advertise<sensor_msgs::Range>(topic_name, 1);

      char ctrl_pin_str[22] = "";
      if (sonars)
        sprintf(ctrl_pin_str, "control pin [DIO%d], ", (*this)[i].ctrl_pin);
      ROS_DEBUG("%s %d: input pin [A%d], %stopic [%s], frame [%s]", sonars?"Sonar":"IR sensor", i,
                (*this)[i].input_pin, ctrl_pin_str, (*this)[i].pub.getTopic().c_str(),
                (*this)[i].msg.header.frame_id.c_str());
    }

    return true;
  }

  /**
   * Converts to voltage, linearizes, filters (KF) and publish the full array of range sensors.
   * @param readings Raw values for all sensors ranging from 0 to 1023
   * @return False if there's any error. True otherwise.
   */
  bool ArduinoNode::RangersList::read(uint16_t* readings)
  {
    for (unsigned int i = 0; i < this->size(); i++)
    {
      double v, r;
      if (this->sonars == true)
      {
        // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V), and then
        // voltage to range multiplying by 25.4/9.8, as sonar doc claims that it reports ~9.8mV/in
        v = readings[i] * (5.0 / 1023.0);
        r = v * 2.59183673469;
        r += (0.06 + r / 40.0);  // XXX hackish compensation empirically devised
                                 // TODO: estimate a polynom as on IR sensors
      }
      else
      {
        // Linearize voltage and convert to range for IR sensors. The 6th grade polynomial's
        // coefficients where calculated by feeding real data to Matlab function polyfit
        v = readings[i];  // TODO: not good... I need new coefficients or reuse old ones
        r = 1.06545479706866e-15*pow(v, 6) - 2.59219822235705e-12*pow(v, 5)
          + 2.52095247302813e-09*pow(v, 4) - 1.25091335895759e-06*pow(v, 3)
          + 0.000334991560873548*pow(v, 2) - 0.0469975280676629*v + 3.01895762047759;
      }

      // Update and publish range message
      (*this)[i].msg.header.seq = (*this)[i].msg.header.seq + 1;
      (*this)[i].msg.header.stamp = ros::Time::now();
      (*this)[i].msg.range = (*this)[i].updateFilter(r);
      // Bound range with min/max range values     XXX I hate to do this...
      // (see https://github.com/DLu/navigation_layers/issues/14 for an explanation of why is this needed)
      (*this)[i].msg.range =
          std::min(std::max((*this)[i].msg.range, (*this)[i].msg.min_range), (*this)[i].msg.max_range);

      (*this)[i].pub.publish((*this)[i].msg);
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
