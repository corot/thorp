#!/usr/bin/env python

"""
Control simulated cats in gazebo:
 - make them slowly hang around the house
 - send out if they tumble (normally hit by the robot)
Author:
    Jorge Santos
"""

import sys
import copy
import math
import rospy
import random

from geometry_msgs.msg import Twist
from mbf_msgs.srv import CheckPoint, CheckPointRequest, CheckPointResponse
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState

from thorp_toolkit.common import wait_for_sim_time
from thorp_toolkit.geometry import roll, yaw, create_2d_pose, pose3d2str, create_3d_point, translate_pose, quaternion_msg_from_yaw


def close_to_obstacle(pose):
    # check if the given location is away from any non-zero cost in the global costmap
    resp = check_point_srv(point=create_3d_point(pose.position.x, pose.position.y, 0, 'map'),
                           costmap=CheckPointRequest.GLOBAL_COSTMAP)
    if resp.state != CheckPointResponse.FREE or resp.cost > 0:
        return True
    return False


def model_states_cb(msg):
    update_period=0.2  # TODO make decorator!
    if hasattr(model_states_cb, 'last_msg_time') and rospy.get_time() - model_states_cb.last_msg_time < update_period:
        return
    model_states_cb.last_msg_time = rospy.get_time()

    global target_models
    if not target_models:
        # all cats toppled; we can stop listening for model states
        rospy.loginfo("All cats toppled; stop listening for model states")
        model_states_sub.unregister()
        return
    # TODO  maybe pause/resume myself (so once)?  looks like not needed with current throttling
    for index, model_name in enumerate(msg.name):
        if model_name in target_models:
            if abs(roll(msg.pose[index])) > hit_roll_threshold:
                # we consider a cat hit after it rolls by more than hit_roll_threshold (pi/2 by default, i.e. toppled)
                rospy.loginfo("Cat %s toppled at %s! (|roll| > %g)",
                              model_name, pose3d2str(msg.pose[index]), hit_roll_threshold)
                # send toppled cats out of the house and stop tracking them
                pose = create_2d_pose(4.0 + len(target_models), 1.0, math.pi/2.0)
                set_model_state_srv(ModelState(model_name, pose, Twist(), 'map'))
                target_models.remove(model_name)
            elif abs(roll(msg.pose[index])) > 0.1:
                # maybe hit? stop moving
                ###TODO not needed, I tink    set_model_state_srv(ModelState(model_name, msg.pose[index], Twist(), 'map'))
                pass
            elif prowling_step > 0.0:
                # hanging around: set small linear speed...
                new_pose = copy.deepcopy(msg.pose[index])
                translate_pose(new_pose, prowling_step, 'x')
                step_mult = 1.0
                while close_to_obstacle(new_pose):  ##mm....  se quedan todos atrapados aqi,,,   TODO BETER
                    #TODO  hostia puta,,,,,  close_to_obstacle -> true cuando el robot ve al gato!!!!
                    #  TODO   gonna need a contact sensor and change direction on contact (ignore floor)
                    # ...but changing directions whenever we approach an obstacle
                    new_pose = copy.deepcopy(msg.pose[index])
                    new_pose.orientation = quaternion_msg_from_yaw(random.uniform(-math.pi, math.pi))
                    translate_pose(new_pose, prowling_step * step_mult, 'x')
                    rospy.loginfo('close --> new yaw  %f    %f', yaw(new_pose.orientation), step_mult)
                    step_mult += 0.25
                #set_model_state_pub.publish(ModelState(model_name, new_pose, Twist(), 'map'))
                set_model_state_srv(ModelState(model_name, new_pose, Twist(), 'map'))


if __name__ == "__main__":
    rospy.init_node("cats_controller")

    if len(sys.argv) < 2:
        rospy.logwarn("No target models provided; nothing to do")
        print("Usage: cats_controller.py <string of space-separated cat names>")
        sys.exit(0)
    target_models = sys.argv[1]
    if type(target_models) != list:
        target_models = target_models.split()

    prowling_speed = Twist()  # TODO can get frequency?  or maybe throttle to save CPU and use to calc step
    prowling_step = rospy.get_param('~prowling_step', 0.0)
    hit_roll_threshold = rospy.get_param('~hit_roll_threshold', math.pi / 2.0)
    set_model_state_srv = rospy.ServiceProxy("gazebo/set_model_state", SetModelState, persistent=True)
    set_model_state_srv.wait_for_service(30)

    # we also need a publisher to update the pose continuously; we don't use it to set the pose cause is not reliable
    set_model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=5)

    # we will use MBF's check pose service to ensure that the spawned surfaces are in open spaces
    check_point_srv = rospy.ServiceProxy('move_base_flex/check_point_cost', CheckPoint, persistent=True)
    check_point_srv.wait_for_service(10)

    # wait for clock to start before listening for models (we unpause simulation after spawning the robot)
    wait_for_sim_time()
    model_states_sub = rospy.Subscriber("gazebo/model_states", ModelStates, model_states_cb, queue_size=1)

    rospy.spin()
