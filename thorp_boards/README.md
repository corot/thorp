# Thorp's boards

Software that interfaces with Thorp's boards:

  * **Arduino Mega 2560** for reading most ranger sensors
  * **Robotis OpenCM9.04** for controlling the arm and interface additional sensors on the top plate

## Arduino

It interfaces with the half-ring of sonars and the four backward pointing IR sensors. Publishes:
  * Static tf from base frame to ring center 
  * Static tfs for the four IR sensors 
  * Sonar readings as a single sensor_msgs/LaserScan
  * IR readings as four sensor_msgs/Range

### Prerequirements

To interface with Thorp's Arduino Mega 2560 board, flash the provided firmware. Note that analog and trigger
pins are hardcoded, so probably you need to adapt the first lines of the firmware to your needs.
Tehn, you must install the Arduino udev rule to make the board available on the default port "/dev/arduino":

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