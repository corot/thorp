THORP
=====

Software for a lowcost mobile manipulation: a TurtleBot2 with an arm, a second 3D camera, and some extra junk

Example demo
------------

[Turtlebot arm object manipulation demo](https://github.com/corot/turtlebot_arm/tree/indigo-devel/turtlebot_arm_object_manipulation) ( [video](https://drive.google.com/file/d/0BzYjlgbSZJSWaVRVQmVKTVczY00/view?usp=sharing)).

Bringup
-------

#### Pre-requisite ####

```
sudo pip install -U yujin_tools
```

#### Installation ####

```
yujin_init_workspace -j10 --track=indigo ~/thorp https://raw.github.com/yujinrobot/kobuki-x/hydro-devel/thorp_bringup/thorp_bringup.rosinstall
cd ~/thorp
source /opt/ros/indigo/setup.bash
rosdep update
rosdep install --from-paths src /opt/ros/indigo --ignore-src --rosdistro indigo -y
yujin_init_build . -u /opt/ros/indigo
yujin_make
```

#### Prepare hardware ####

Arm, sonars and IR sensors require a bit of extra work. Check their [README](https://github.com/corot/thorp/tree/master/thorp_boards) for instructions.
