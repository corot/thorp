#!/bin/bash

cd /home/jorge/catkin_ws/thorp/src/thorp/thorp_navigation/launch/includes/
cp move_base_flex.launch.xml move_base_flex.launch.xml.tmp
cp move_base_flex.launch.xml.debug move_base_flex.launch.xml
cp move_base_flex.launch.xml.tmp move_base_flex.launch.xml.debug
rm move_base_flex.launch.xml.tmp
rosnode kill /move_base_flex