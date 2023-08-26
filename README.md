THORP
=====

Software for a low-cost mobile manipulation: a TurtleBot2 with an arm, a second 3D camera, and some extra junk.

Example apps
------------

### Simulated on Gazebo

Cat hunter: explore the entire environment searching for cats, and attack them whenever found.
```
roslaunch thorp_simulation cat_hunter.launch cat_models:='cat_orange cat_black'
```

[![Cat hunter](https://user-images.githubusercontent.com/322610/121386461-fb111980-c984-11eb-92e3-0e59c733f789.png)](https://youtu.be/ieW3BQabwLo "Cat hunter")

Object gatherer: explore the entire environment gathering all tabletop objects of the requested types.
```
roslaunch thorp_simulation object_gatherer.launch object_types:='cube circle pentagon'
```
[![Object gatherer](https://user-images.githubusercontent.com/322610/121390421-51338c00-c988-11eb-9bbf-e89d588018cb.png)](https://youtu.be/tneMk6kRPHU "Object gatherer")


### Real robot (much older)

[Turtlebot arm object manipulation demo](https://github.com/corot/turtlebot_arm/tree/melodic-devel/turtlebot_arm_object_manipulation) ([video](https://youtu.be/GA-PLYbH06o)).

Bringup
-------

#### Pre-requisite ####

Install wstool:

```
sudo apt-get install python3-wstool
```
or when that is not possible, fall back to pip:
```
sudo pip install -U wstool
```

#### Installation ####

```
mkdir ~/thorp
cd ~/thorp
wstool init src https://raw.githubusercontent.com/corot/thorp/master/thorp_bringup/docker/thorp.rosinstall
touch src/gazebo-pkgs/CATKIN_IGNORE
ln -sr src/gazebo-pkgs/gazebo_grasp_plugin src
ln -sr src/gazebo-pkgs/gazebo_version_helpers src
source /opt/ros/noetic/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
catkin build
pip install --upgrade bidict ratelimit tk
```

#### Prepare hardware ####

Arm, sonars and IR sensors require a bit of extra work. Check their
[README](https://github.com/corot/thorp/tree/master/thorp_boards) for instructions.

### Troubleshooting guide
- Cannot connect with Kobuki base:
  - Ensure you have installed the udev rule as explained in the
    [kobuki_ftdi](https://github.com/yujinrobot/kobuki_core/tree/noetic/kobuki_ftdi) package 
- Cannot connect with arduino, OpenCM or USB2AX:
  - Ensure you have installed the udev rules as explained in the
    [thorp_boards](https://github.com/corot/thorp/tree/master/thorp_boards) package
- Gazebo error in REST request:
  - fix explained [here](https://answers.gazebosim.org//question/22263/error-in-rest-request-for-accessing-apiignitionorg)
- Tensorflow not working:
  - [Install tensorflow and right cuda version](https://www.tensorflow.org/install/gpu)

### Docker image

Available in [dockerhub](https://hub.docker.com/repository/docker/corot/thorp)

:warning: Manipulation in noetic image doesn't work properly because of [this MoveIt! issue](https://github.com/ros-planning/moveit/issues/3007).
Re-detected objects will be considered as new ones, and so collide with the previously added to planning scene.

```
docker pull corot/thorp:melodic
```

To build locally:

```
cd ~/thorp/thorp_bringup/docker
docker build -t thorp .
```

Run with rocker:

```
pip install rocker
rocker --privileged --pulse --nvidia --x11 thorp
```

or the pulled image:

```
rocker --privileged --pulse --nvidia --x11 corot/thorp:melodic
```

Once inside the docker, you can run any of the simulation apps, e.g.:

```
roslaunch thorp_simulation thorp_stage.launch
```
