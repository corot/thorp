ARG ROS_DISTRO=melodic

FROM ros:$ROS_DISTRO

RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-ros-base

RUN pip install bidict

# create catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
COPY thorp.rosinstall .
RUN wstool init -j10 . thorp.rosinstall

# delete non-relevant pkgs from cloned repositories
RUN rm -rf gazebo-pkgs/gazebo_state_plugins \
           gazebo-pkgs/gazebo_test_tools \
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
           turtlebot_interactions/turtlebot_interactions \
           turtlebot_interactions/turtlebot_interactive_markers \
           turtlebot_interactions/turtlebot_rviz_launchers \
           turtlebot_arm/turtlebot_arm \
           turtlebot_arm/turtlebot_arm_block_manipulation \
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

# build catkin workspace
RUN rosdep update --rosdistro=$ROS_DISTRO && \
    rosdep install -y --rosdistro=$ROS_DISTRO --from-paths . --ignore-src \
                   --skip-keys='create_dashboard create_description rocon_app_manager kobuki_rapps turtlebot_capabilities'

RUN catkin_make
#RUN source /catkin_ws/devel/setup.bash

# link gazebo models
ENV GAZEBO_MODEL_PATH=/catkin_ws/thorp/src/thorp/thorp_simulation/worlds/gazebo/models

#  gazebo_ros_depth_camera   probably need my local change

ENV APP_NAME=$APP_NAME
ENV ARGUMENTS=$ARGUMENTS
RUN echo "RUNNING roslaunch thorp_simulation $APP_NAME.launch $ARGUMENTS"

CMD roslaunch thorp_simulation $APP_NAME.launch $ARGUMENTS

# run with -e APP_NAME=object_gatherer -e ARGUMENTS="viz_smach:=false object_types:='triangle'"
