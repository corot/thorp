# Thorp's boards

Software that interfaces with Thorp's boards:

  * **Arduino Mega 2560** for reading most ranger sensors
  * **Robotis OpenCM9.04** for controlling the rocket launcher and reading additional sensors on the top plate
  * **USB2AX** for controlling the arm

## Arduino

It interfaces with the half-ring of sonars and the four backward pointing IR sensors. It publishes:
  * 11 sonar readings as sensor_msgs/Range messages
  * 4 IR sensor readings as sensor_msgs/Range messages

To interface with Thorp's Arduino Mega 2560 board, flash the provided firmware. You can use the Arduino IDE
or use make commands (see [this document for details](http://ed.am/dev/make/arduino-mk)). Note that analog
and trigger pins are hardcoded, so probably you need to adapt the first lines of the firmware to your needs.
The default port "/dev/arduino".

## OpenCM9.04

To interface with Thorp's Robotis OpenCM9.04 board we use the [arbotix packages](http://wiki.ros.org/arbotix).
Before, flash the firmware provided on src/opencm/firmware directory, as explained in the README file there.    
The default port "/dev/opencm":

## USB2AX

The default port "/dev/USB2AX":

## UDev rules

To make the boards available on the default ports, you must install a udev rule for each board: 
```
$ roscd thorp_boards
$ sudo cp resources/58-arduino.rules /etc/udev/rules.d
$ sudo cp resources/59-opencm.rules /etc/udev/rules.d
$ sudo cp resources/60-usb2ax.rules /etc/udev/rules.d
```
and then replug the USB cables.
