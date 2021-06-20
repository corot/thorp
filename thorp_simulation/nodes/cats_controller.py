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
import ratelimit

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates, ContactsState
from gazebo_msgs.srv import SetModelState

from thorp_toolkit.common import wait_for_sim_time
from thorp_toolkit.geometry import roll, norm_angle, create_2d_pose, translate_pose, pose3d2str, quaternion_msg_from_yaw


class CatsController:
    def __init__(self):
        self.target_models = rospy.get_param('~target_models', 'cat_orange cat_black').split()
        self.alive_cats = {}  # dictionary to store relevant events for still-alive cats
        self.killed_cats = []  # casualties record

        # update period is hardcoded to 0.25 cause the throttling decorator is created before starting the node,
        # and so I cannot read if from the parameter server  TODO: find a better solution
        self.prowling_step = rospy.get_param('~prowling_step', 0.01)
        self.hit_sock_duration = rospy.get_param('~hit_sock_duration', 2.0)
        self.hit_roll_threshold = rospy.get_param('~hit_roll_threshold', math.pi / 3.0)
        self.set_model_state_srv = rospy.ServiceProxy("gazebo/set_model_state", SetModelState, persistent=True)
        self.set_model_state_srv.wait_for_service(30)

        # subscribe to contact events for all cats
        self.contacts_sub = rospy.Subscriber('gazebo/contacts', ContactsState, self.contacts_cb, queue_size=5)

        # wait for clock to start before listening for models (we unpause simulation after spawning the robot)
        wait_for_sim_time()
        self.model_states_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1)

    def handle_contact(self, involved1, involved2, contact):
        if involved1 in self.alive_cats.keys():
            if involved2.startswith('rocket'):
                self.alive_cats[involved1]['last_hit'] = rospy.get_time()
            else:
                self.alive_cats[involved1]['contact'] = contact

    def contacts_cb(self, msg):
        for contact in msg.states:
            involved1 = contact.collision1_name.split('::')[0]
            involved2 = contact.collision2_name.split('::')[0]
            self.handle_contact(involved1, involved2, contact)
            self.handle_contact(involved2, involved1, contact)

    @ratelimit.limits(calls=1, period=0.25, raise_on_limit=False)
    def model_states_cb(self, msg):
        if self.killed_cats and not self.alive_cats:
            # all cats toppled; we can stop listening for model states
            rospy.loginfo("All cats toppled; stop listening for model states")
            self.model_states_sub.unregister()
            return
        # TODO  maybe pause/resume myself (so once)?  looks like not needed with current throttling
        for index, model_name in enumerate(msg.name):
            if model_name in self.alive_cats:
                if abs(roll(msg.pose[index])) > self.hit_roll_threshold:
                    # we consider a cat toppled after it rolls by more than hit_roll_threshold
                    # TODO: maybe better, use body/head contacts with ground_plane
                    rospy.loginfo("Cat %s toppled at %s! (|roll| > %g)",
                                  model_name, pose3d2str(msg.pose[index]), self.hit_roll_threshold)
                    # send toppled cats out of the house and stop tracking them
                    pose = create_2d_pose(4.0 + len(self.killed_cats), 1.0, math.pi / 2.0)
                    self.set_model_state_srv(ModelState(model_name, pose, Twist(), 'map'))
                    self.killed_cats.append(model_name)
                    del self.alive_cats[model_name]
                elif rospy.get_time() - self.alive_cats[model_name]['last_hit'] < self.hit_sock_duration:
                    # hit, so stop moving; admittedly, not the most realistic for a cat
                    pass
                elif self.prowling_step > 0.0:
                    # hanging around: perform small steps but changing directions whenever we touch anything
                    new_pose = copy.deepcopy(msg.pose[index])
                    contact = self.alive_cats[model_name]['contact']
                    if contact:
                        # use inv contact direction to limit the possible orientations to those away from the contact
                        contact_inv_dir = math.atan2(contact.contact_normals[0].y, contact.contact_normals[0].x)
                        new_yaw_bound1 = norm_angle(contact_inv_dir - math.pi / 3.0)
                        new_yaw_bound2 = norm_angle(contact_inv_dir + math.pi / 3.0)
                        new_yaw = random.uniform(new_yaw_bound1, new_yaw_bound2)
                        new_pose.orientation = quaternion_msg_from_yaw(new_yaw)
                        rospy.loginfo("%s: contact from %f; new direction to %f", model_name, contact_inv_dir, new_yaw)
                        self.alive_cats[model_name]['contact'] = None
                    translate_pose(new_pose, self.prowling_step, 'x')
                    self.set_model_state_srv(ModelState(model_name, new_pose, Twist(), 'map'))
            elif model_name not in self.killed_cats:
                # first time to see this model; keep track of it if listed under target models
                # (individual instances are numbered _0, _1 and so by the model spawner)
                if model_name in self.target_models or '_'.join(model_name.split('_')[:-1]) in self.target_models:
                    self.alive_cats[model_name] = {'last_hit': -1.0, 'contact': None}


if __name__ == "__main__":
    rospy.init_node("cats_controller")

    CatsController()
    rospy.spin()
