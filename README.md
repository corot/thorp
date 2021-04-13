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
wstool init src https://raw.githubusercontent.com/corot/thorp/master/thorp_bringup/docker/thorp.rosinstall
touch src/gazebo-pkgs/CATKIN_IGNORE
ln -sr src/gazebo-pkgs/gazebo_grasp_plugin src
ln -sr src/gazebo-pkgs/gazebo_version_helpers src
source /opt/ros/melodic/setup.bash
rosdep update
rosdep install --from-paths src /opt/ros/melodic --ignore-src --rosdistro melodic -y
catkin build
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

```
docker pull corot/thorp
```

To build locally:

```
cd ~/thorp/thorp_bringup/docker
docker build -t thorp .
```

Run with rocker:
```
rocker --pulse --nvidia --x11 thorp
```
or
```
rocker --pulse --nvidia --x11 corot/thorp
```
