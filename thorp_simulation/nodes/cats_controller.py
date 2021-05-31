#!/usr/bin/env python

"""
Control simulated cats in gazebo:
 - make them slowly hang around the house (TODO)
 - send out if they tumble (normally hit by the robot)
Author:
    Jorge Santos
"""

import sys
import copy
import math
import rospy

from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelState, ModelStates

from thorp_toolkit.common import wait_for_sim_time
from thorp_toolkit.geometry import quaternion_msg_from_yaw, roll, yaw, create_2d_pose, pose3d2str


def model_states_cb(msg):
    global target_models
    if not target_models:
        # all cats done; we can stop listening for model states
        rospy.loginfo("All cats done; stop listening for model states")
        model_states_sub.unregister()
        return
    for index, model_name in enumerate(msg.name):
        if model_name in target_models:
            if abs(roll(msg.pose[index])) > hit_roll_threshold:
                target_models.remove(model_name)
                rospy.loginfo("Cat %s at %s hit! (|roll| > %g)",
                              model_name, pose3d2str(msg.pose[index]), hit_roll_threshold)
                pose = create_2d_pose(4.0 + len(target_models), 1.0, 0.0)
                model_state_pub.publish(ModelState(model_name, pose, Twist(), 'map'))


if __name__ == "__main__":
    rospy.init_node("cats_controller")

    model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=1)
    hit_roll_threshold = rospy.get_param('~hit_roll_threshold', math.pi / 2.0)

    if len(sys.argv) < 2:
        rospy.logwarn("No target models provided; nothing to do")
        print("Usage: cats_controller.py <string of space-separated cat names>")
        sys.exit(0)
    target_models = sys.argv[1]
    if type(target_models) != list:
        target_models = target_models.split()

    # wait for clock to start before listening for models (we unpause simulation after spawning the robot)
    wait_for_sim_time()
    model_states_sub = rospy.Subscriber("gazebo/model_states", ModelStates, model_states_cb, queue_size=1)

    rospy.spin()
