# Thorp's arm control package

Software that interfaces with Thorp's arm through [dynamixel_motor](http://wiki.ros.org/dynamixel_motor) and the USB2AX interface.

***TODO:** try the [official Robotis packages](https://github.com/ROBOTIS-GIT/DynamixelSDK) instead*

### Prerequirements

To interface with Thorp's arm you must install the USB2AX interface udev rule to make it available on the default port "/dev/USB2AX":

    $ roscd thorp_manipulation
    $ sudo cp resources/60-usb2ax.rules /etc/udev/rules.d
 
and then replug the USB cable.
