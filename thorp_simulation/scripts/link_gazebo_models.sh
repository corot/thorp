#!/bin/bash

cd
cd .gazebo/models
ln -s ~/catkin_ws/thorp/src/thorp/thorp_simulation/worlds/gazebo/models/cat_orange .
mkdir doll_table
cd doll_table
ln -s ~/catkin_ws/perception/src/rail_collada_models/meshes/original_models/doll_furniture .
