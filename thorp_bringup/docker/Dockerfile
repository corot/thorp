ARG ROS_DISTRO=melodic

FROM ros:$ROS_DISTRO

# Update ROS keys (see https://discourse.ros.org/t/new-gpg-keys-deployed-for-packages-ros-org/9454)
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116 && apt update

# nvidia-docker hooks
# LABEL com.nvidia.volumes.needed="nvidia_driver"
# ENV PATH /usr/local/nvidia/bin:${PATH}
# ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

SHELL ["/bin/bash", "-c"]

# install ROS base and minimal build dependencies
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-ros-base python-pip python-catkin-tools build-essential wget nano git

# add packages.osrfoundation.org APT to install the latest gazebo9
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && apt update

# reduce log verbosity (RAIL segmentation is called continuously for some apps)
RUN echo 'log4j.logger.ros.rail_mesh_icp=WARN' >> /opt/ros/$ROS_DISTRO/share/ros/config/rosconsole.config && \
    echo 'log4j.logger.ros.rail_segmentation=ERROR' >> /opt/ros/$ROS_DISTRO/share/ros/config/rosconsole.config

# create catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
COPY thorp.rosinstall .
RUN wstool init -j10 . thorp.rosinstall

# cleanup git history to reduce image size  (got from https://gist.github.com/facelordgists/80e868ff5e315878ecd6)
RUN find . \( -name ".git" -o -name ".gitignore" -o -name ".gitmodules" -o -name ".gitattributes" \) -exec rm -rf -- {} +

# delete non-relevant pkgs from cloned repositories
RUN rm -rf navigation/amcl \
           navigation/fake_localization \
           navigation/map_server \
           navigation/move_slow_and_clear \
           navigation/navfn \
           navigation/voxel_grid \
           navigation/base_local_planner \
           navigation/clear_costmap_recovery \
           navigation/dwa_local_planner \
           navigation/global_planner \
           navigation/move_base \
           navigation/nav_core \
           navigation/navigation \
           navigation/rotate_recovery \
           navigation_experimental/assisted_teleop \
           navigation_experimental/goal_passer \
           navigation_experimental/navigation_experimental \
           navigation_experimental/pose_base_controller \
           navigation_experimental/sbpl_recovery \
           navigation_experimental/sbpl_lattice_planner \
           navigation_experimental/twist_recovery \
           navigation_layers/navigation_layers \
           navigation_layers/social_navigation_layers \
           gazebo-pkgs/gazebo_state_plugins \
           gazebo-pkgs/gazebo_test_tools \
           gazebo_ros_pkgs/docs \
           gazebo_ros_pkgs/gazebo_dev \
           gazebo_ros_pkgs/gazebo_msgs \
           gazebo_ros_pkgs/gazebo_ros \
           gazebo_ros_pkgs/gazebo_ros_control \
           gazebo_ros_pkgs/gazebo_ros_pkgs \
           kobuki/kobuki \
           kobuki/kobuki_controller_tutorial \
           kobuki/kobuki_random_walker \
           kobuki/kobuki_testsuite \
           kobuki/kobuki_capabilities \
           kobuki/kobuki_rapps \
           kobuki_desktop/kobuki_desktop \
           kobuki_desktop/kobuki_gazebo \
           kobuki_desktop/kobuki_qtestsuite \
           kobuki_desktop/kobuki_rviz_launchers \
           turtlebot/turtlebot \
           turtlebot/turtlebot_capabilities \
           turtlebot_apps/turtlebot_apps \
           turtlebot_apps/turtlebot_actions \
           turtlebot_apps/turtlebot_calibration \
           turtlebot_apps/turtlebot_follower \
           turtlebot_apps/turtlebot_rapps \
           turtlebot_interactions/turtlebot_interactions \
           turtlebot_interactions/turtlebot_interactive_markers \
           turtlebot_interactions/turtlebot_rviz_launchers \
           turtlebot_arm/turtlebot_arm \
          # turtlebot_arm/turtlebot_arm_block_manipulation   TODO: remove once I stop using on thorp_smach \
           turtlebot_arm/turtlebot_arm_kinect_calibration \
           turtlebot_arm/turtlebot_arm_moveit_demos \
           turtlebot_arm/turtlebot_arm_object_manipulation \
           yujin_ocs/yujin_ocs \
           yujin_ocs/yocs_ar_pair_tracking \
           yujin_ocs/yocs_diff_drive_pose_controller \
           yujin_ocs/yocs_localization_manager \
           yujin_ocs/yocs_navi_toolkit \
           yujin_ocs/yocs_waypoints_navi \
           yujin_ocs/yocs_ar_marker_tracking \
           yujin_ocs/yocs_joyop \
           yujin_ocs/yocs_rapps \
           yujin_ocs/yocs_virtual_sensor \
           yujin_ocs/yujin_ocs \
           yujin_ocs/yocs_ar_pair_approach \
           yujin_ocs/yocs_keyop \
           yujin_ocs/yocs_navigator \
           yujin_ocs/yocs_waypoint_provider

# install all dependencies; skip all keys that rosdep cannot find (and are not required)
RUN rosdep update --rosdistro=$ROS_DISTRO && \
    rosdep install -y -r --rosdistro=$ROS_DISTRO --from-paths . --ignore-src \
                   --skip-keys='create_node create_dashboard create_description \
                                rocon_app_manager kobuki_rapps kobuki_capabilities turtlebot_capabilities \
                                stdr_gui stdr_robot stdr_server astra_launch realsense_camera zeroconf_avahi'
RUN apt clean

# bidict 0.18.3 is the last version compatible with Python 2.7
RUN pip install --upgrade bidict==0.18.3 ratelimit matplotlib tensorflow

# build catkin workspace and remove build to reduce image size
WORKDIR /catkin_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN rm -r build

# setup runtime environment
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
ENV ROSCONSOLE_FORMAT='[${node} ${severity} ${time}]: ${message}'

# fix Gazebo error in REST request (https://answers.gazebosim.org//question/22263/error-in-rest-request-for-accessing-apiignitionorg)
COPY patch_ignition.yaml /root/.ignition/fuel/config.yaml

# path to gazebo models
ENV GAZEBO_MODEL_PATH=/catkin_ws/src/thorp/thorp_simulation/worlds/gazebo/models

# ENV APP_NAME=$APP_NAME
# ENV ARGUMENTS=$ARGUMENTS
# RUN echo "RUNNING roslaunch thorp_simulation $APP_NAME.launch $ARGUMENTS"


# CMD source devel/setup.bas && gazebo
#CMD source devel/setup.bas && roslaunch thorp_simulation $APP_NAME.launch $ARGUMENTS

# run with -e APP_NAME=object_gatherer -e ARGUMENTS="viz_smach:=false object_types:='pentagon' start_delay:=20"
