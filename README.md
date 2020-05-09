THORP
=====

Software for a low-cost mobile manipulation: a TurtleBot2 with an arm, a second 3D camera, and some extra junk.

Example demo
------------

[Turtlebot arm object manipulation demo](https://github.com/corot/turtlebot_arm/tree/melodic-devel/turtlebot_arm_object_manipulation) ([video](https://drive.google.com/file/d/0BzYjlgbSZJSWaVRVQmVKTVczY00/view?usp=sharing)).

Bringup
-------

#### Pre-requisite ####

Install wstool:

```
sudo apt-get install python-wstool
```
or when that is not possible, fall back to pip:
```
sudo pip install -U wstool
```

#### Installation ####

```
mkdir ~/thorp
cd ~/thorp
wstool init src https://raw.githubusercontent.com/corot/thorp/master/thorp.rosinstall
touch src/gazebo-pkgs/CATKIN_IGNORE
ln -sr src/gazebo-pkgs/gazebo_grasp_plugin src
ln -sr src/gazebo-pkgs/gazebo_version_helpers src
source /opt/ros/melodic/setup.bash
rosdep update
rosdep install --from-paths src /opt/ros/melodic --ignore-src --rosdistro melodic -y
catkin build
```

#### Prepare hardware ####

Arm, sonars and IR sensors require a bit of extra work. Check their [README](https://github.com/corot/thorp/tree/master/thorp_boards) for instructions.
