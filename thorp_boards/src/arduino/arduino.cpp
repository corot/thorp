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
             : nh(n), wrong_readings(0), is_connected(false) {}

  ArduinoNode::~ArduinoNode() {}

  bool ArduinoNode::spin()
  {
    unsigned long long i = 0;
    ros::Rate rate(20.0);

  //  scan.header.frame_id = ir_frame_id;
int kk = 0;
    while (ros::ok())
    {
      if (is_connected)
      {
        if ((readSonars() == false) || (readIrSensors() == false))
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
  //      if (kk%100 == 10){
        triggerSonars();
      //  ROS_WARN("TRIGGER");}
     //   kk++;
      }
      else {
        is_connected = connect();
      }
      ros::spinOnce();
      rate.sleep();
    }

    return true;
  }

  /**
   * Trigger one reading on all sonars
   * @return True if there aren't IO failures. False otherwise.
   */
  bool ArduinoNode::triggerSonars()
  {
    for (unsigned int i = 0; i < sonars.size(); i++)
    {
      // Set ranger control pin (RX) to HIGH for at least 20μS to start one range reading
      if (! sonars[i].gpio_driver->set(true))
      {
        ROS_WARN("Arduino interface: GPIO driver failed to set ranger %d control pin to HIGH", i);
        return false;
      }
      ros::Duration(0.0001); // 100 μs
      if (! sonars[i].gpio_driver->set(false))
      {
        ROS_WARN("Arduino interface: GPIO driver failed to set ranger %d control pin to LOW", i);
        return false;
      }
    }
    return true;
  }

  /**
   * Reads, linearizes, filters (KF) and publish the full array of sonars.
   * @return False if we must stop node after many wrong readings. True otherwise.
   */
  bool ArduinoNode::readSonars()
  {
    for (unsigned int i = 0; i < sonars.size(); i++)
    {
      // Read ranger i digitized voltage
      uint32_t reading = sonars[i].adc_driver->read();

      if (reading == 0)  // we never read 0 in normal operation
        return false;

      // Convert voltage to range and fill range/intensity vectors
      double v = reading/1000000.0;
      double r = v*2.59183673469;  // or 25.4/9.8, as sonar doc claims that it reports ~9.8mV/in

      /// TODO not bounding min/max ranges!!!
//      scan.ranges[sonars.size() - (i + 1)] = distanceKF <= maximum_range ? distanceKF + sonar_ring_rad : infinity_range;
//      scan.intensities[sonars.size() - (i + 1)] = v;

      scan.ranges[i] = sonars[i].updateFilter(r) + sonar_ring_rad;
      scan.intensities[i] = r + sonar_ring_rad; ///////////////v;
    }

    // Update and publish laser scan message
    scan.header.seq++;
    scan.header.stamp = ros::Time::now();

    sonars_pub.publish(scan);

    return true;
  }

  /**
   * Reads, linearizes, filters (KF) and publish the full array of IR sensors.
   * @return False if we made a wrong reading (Arduino returned 0). True otherwise.
   */
  bool ArduinoNode::readIrSensors()
  {
    for (unsigned int i = 0; i < irSensors.size(); i++)
    {
      // Read ranger i digitized voltage
      uint32_t reading = irSensors[i].adc_driver->read();

      if (reading == 0)  // we never read 0 in normal operation
        return false;

      // Linearize voltage and convert to range
      double v = reading/5000.0;
      double r = 1.06545479706866e-15*pow(v, 6) - 2.59219822235705e-12*pow(v, 5) + 2.52095247302813e-09*pow(v, 4)
               - 1.25091335895759e-06*pow(v, 3) + 0.000334991560873548*pow(v, 2) - 0.0469975280676629*v + 3.01895762047759;

      // Update and publish range message
      irSensors[i].msg.header.seq = irSensors[i].msg.header.seq + 1;
      irSensors[i].msg.header.stamp = ros::Time::now();
      irSensors[i].msg.range = irSensors[i].updateFilter(r); /// TODO not bounding min/max ranges!!!  distanceKF <= irSensors[i].msg.max_range ? distanceKF : irSensors[i].infinity_range;

      irSensors[i].pub.publish(irSensors[i].msg);
    }
    return true;
  }

  bool ArduinoNode::init()
  {
    // Parameters; note that for all maps, the default value is an empty list, what is invalid
    std::string default_frame("/sonars_link");
    std::string default_port("/dev/arduino");

    ///////////////////////////
    // General configuration //
    ///////////////////////////
    nh.param("arduino_node/read_frequency",   read_frequency, 20.0);
    nh.param("arduino_node/arduino_port",     arduino_port, default_port);

    int    rangers_count;
    double range_variance;
    double maximum_range;
    double minimum_range;
    double infinity_range;  // TODO not used;  remove????
    std::string frame_id;   // Frame id for the output sonars' laser scan
    XmlRpc::XmlRpcValue input_pins_map;
    XmlRpc::XmlRpcValue ctrl_pins_map;

    ///////////////////////////////
    // Sonars ring configuration //
    ///////////////////////////////
    nh.param("arduino_node/sonars/rangers_count",  rangers_count,  11);
    nh.param("arduino_node/sonars/range_variance", range_variance,  0.025);
    nh.param("arduino_node/sonars/maximum_range",  maximum_range,   6.5);
    nh.param("arduino_node/sonars/minimum_range",  minimum_range,   0.1);
    nh.param("arduino_node/sonars/infinity_range", infinity_range, 20.0);
    nh.param("arduino_node/sonars/ring_radius",    sonar_ring_rad,  0.155);
    nh.param("arduino_node/sonars/frame_id",       frame_id, default_frame);
    nh.param("arduino_node/sonars/ctrl_pins_map",  ctrl_pins_map,  ctrl_pins_map);
    nh.param("arduino_node/sonars/input_pins_map", input_pins_map, input_pins_map);

    // Validate maps: missconfiguration here will almost surely
    // make something fail, so we abort if something is wrong
    if ((validMap(ctrl_pins_map, "sonars control pins",
                  rangers_count, XmlRpc::XmlRpcValue::TypeInt) == false) ||
        (validMap(input_pins_map, "sonars input pins",
                  rangers_count, XmlRpc::XmlRpcValue::TypeInt) == false))
    {
      return false;
    }

    sonars.resize(rangers_count, Sonar());

    // Fill all fields on sonars array except the Bosch drivers, because
    // we must be connected to the Arduino before calling their constructor
    for (unsigned int i = 0; i < sonars.size(); i++)
    {
      sonars[i].last_range = maximum_range;
      sonars[i].Q = 0.001;
      sonars[i].R = range_variance; //0.0288;
      sonars[i].P = sonars[i].R;
      sonars[i].ctrl_pin = static_cast<int>(ctrl_pins_map[i]);
      sonars[i].input_pin = static_cast<int>(input_pins_map[i]);

      ROS_DEBUG("Sonar %d read at analog input A%d, and is triggered by pin %d",
                i, sonars[i].input_pin, sonars[i].ctrl_pin);
    }

    // Fill all constant values on sonars' laser scan
    scan.ranges.resize(rangers_count, 0.0);
    scan.intensities.resize(rangers_count, 0.0);

    scan.angle_min = -M_PI/2.0;
    scan.angle_max = +M_PI/2.0;
    scan.angle_increment = M_PI/10.0;
    scan.scan_time = 1.0 / read_frequency;
    scan.range_min = 0.10 + sonar_ring_rad;
    scan.range_max = 11.00 + sonar_ring_rad;
    scan.header.frame_id = frame_id;

    // Sonars' laser scan publisher
    sonars_pub = nh.advertise<sensor_msgs::LaserScan>("sonars", 1);

    //////////////////////////////
    // IR sensors configuration //
    //////////////////////////////
    XmlRpc::XmlRpcValue frame_ids_map;
    XmlRpc::XmlRpcValue topic_names_map;
    input_pins_map.clear();

    nh.param("arduino_node/infrared/rangers_count",   rangers_count,    4);
    nh.param("arduino_node/infrared/range_variance",  range_variance,   0.025);
    nh.param("arduino_node/infrared/maximum_range",   maximum_range,    0.8);
    nh.param("arduino_node/infrared/minimum_range",   minimum_range,    0.1);
    nh.param("arduino_node/infrared/infinity_range",  infinity_range,  10.0);
    nh.param("arduino_node/infrared/input_pins_map",  input_pins_map,  input_pins_map);
    nh.param("arduino_node/infrared/frame_ids_map",   frame_ids_map,   frame_ids_map);
    nh.param("arduino_node/infrared/topic_names_map", topic_names_map, topic_names_map);

    // Validate maps: missconfiguration here will almost surely
    // make something fail, so we abort if something is wrong
    if ((validMap(input_pins_map, "IR input pins",
                  rangers_count, XmlRpc::XmlRpcValue::TypeInt) == false) ||
        (validMap(frame_ids_map, "IR frame ids",
                  rangers_count, XmlRpc::XmlRpcValue::TypeString) == false) ||
        (validMap(topic_names_map, "IR topic names",
                  rangers_count, XmlRpc::XmlRpcValue::TypeString) == false))
    {
      return false;
    }

    irSensors.resize(rangers_count, IrSensor());

    // Fill all fields on IR sensors array except the Bosch drivers, because
    // we must be connected to the Arduino before calling their constructor
    for (unsigned int i = 0; i < irSensors.size(); i++)
    {
      irSensors[i].last_range = maximum_range;
      irSensors[i].Q = 0.001;
      irSensors[i].R = range_variance; //0.0288;
      irSensors[i].P = irSensors[i].R;
      irSensors[i].input_pin = static_cast<int>(input_pins_map[i]);

      // Fill also all constant values on each range message
      irSensors[i].msg.header.frame_id = static_cast<std::string>(frame_ids_map[i]);
      irSensors[i].msg.radiation_type = sensor_msgs::Range::INFRARED;
      irSensors[i].msg.field_of_view = 0.062; // ~3.5 degrees
      irSensors[i].msg.min_range = minimum_range;
      irSensors[i].msg.max_range = maximum_range;

      // We need individual publishers for each sensor
      irSensors[i].pub =
          nh.advertise<sensor_msgs::Range>(static_cast<std::string>(topic_names_map[i]), 1);

      ROS_DEBUG("IR sensor %d: input port [A%d], topic [%s], frame [%s]", i,
                irSensors[i].input_pin, irSensors[i].pub.getTopic().c_str(),
                irSensors[i].msg.header.frame_id.c_str());
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

    // Create an ADC driver (reading range) and a GPIO driver (trigger readings) for every sonar
    for (unsigned int i = 0; i < sonars.size(); i++)
    {
      sonars[i].adc_driver.reset(new AdcDriver(arduino_iface.get(), sonars[i].input_pin));
      sonars[i].gpio_driver.reset(new GpioDriver(arduino_iface.get(), sonars[i].ctrl_pin));

      sonars[i].adc_driver->setReference(5000); // can be also 1100 or 2560
    }

    // Create an ADC driver (reading range) for every IR sensor (Sharp sensors don't
    // need to be triggered; just operate continuously as long as current is provided)
    for (unsigned int i = 0; i < irSensors.size(); i++)
    {
      irSensors[i].adc_driver.reset(new AdcDriver(arduino_iface.get(), irSensors[i].input_pin));
      irSensors[i].adc_driver->setReference(5000); // can be also 1100 or 2560
    }

    ROS_INFO("Arduino interface successfully initialized with %lu sonars and %lu IR sensors",
              sonars.size(), irSensors.size());
    return true;
  }

  bool ArduinoNode::validMap(const XmlRpc::XmlRpcValue& map,
                             const std::string& name, int size, XmlRpc::XmlRpcValue::Type type)
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

