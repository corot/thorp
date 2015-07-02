# Thorp's boards

Software that interfaces with Thorp's boards:

  * **Arduino Mega 2560** for reading most ranger sensors
  * **Robotis OpenCM9.04** for controlling the arm and interface additional sensors on the top plate

## Arduino

It interfaces with the half-ring of sonars and the four backward pointing IR sensors. It publishes:
  * 11 sonar readings as sensor_msgs/Range messages
  * 4 IR sensor readings as sensor_msgs/Range messages

### Prerequirements

To interface with Thorp's Arduino Mega 2560 board, flash the provided firmware. You can use the Arduino IDE
or use make commands (see [this document for details](http://ed.am/dev/make/arduino-mk)). Note that analog
and trigger pins are hardcoded, so probably you need to adapt the first lines of the firmware to your needs.
Then, you must install the Arduino udev rule to make the board available on the default port "/dev/arduino":

    $ roscd thorp_boards
    $ sudo cp resources/58-arduino.rules /etc/udev/rules.d
 
and then replug the USB cable.

## OpenCM9.04

To interface with Thorp's Robotis OpenCM9.04 board we use the [arbotix packages](http://wiki.ros.org/arbotix).
Before, flash the firmware provided on src/opencm/firmware directory, as explained in the README file there.    

As for the Arduino, you must install an udev rule to make the board available on the default port "/dev/opencm":

    $ roscd thorp_boards
    $ sudo cp resources/59-opencm.rules /etc/udev/rules.d
 
and then replug the USB cable.