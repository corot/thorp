#!/usr/bin/env python3

"""
Control simulated cats in Gazebo:
 - make them slowly go around the house, turning away from whatever they touch
 - stop them for a moment when hit by a rocket, so the hit can topple them
 - send them out if they tumble (normally hit by the robot)
Needs the cats' contacts and odometry bridged from Gazebo.
Author:
    Jorge Santos
"""

import math
import random

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts, Entity
from ros_gz_interfaces.srv import SetEntityPose

from thorp_toolkit.geometry import create_2d_pose, pose3d2str, quaternion_msg_from_yaw, translate_pose

UPDATE_PERIOD = 0.1


def tilt(q):
    """ Angle between the z axes of the given orientation and the world """
    return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (q.x ** 2 + q.y ** 2))))


class CatsController(Node):
    def __init__(self):
        super().__init__('cats_controller')
        self.declare_parameter('target_models', ['cat_orange', 'cat_black'])
        self.declare_parameter('prowling_speed', 0.1)
        self.declare_parameter('hit_sock_duration', 1.5)
        self.declare_parameter('outside_x', 0.0)
        self.declare_parameter('outside_y', 0.0)
        self.declare_parameter('hit_tilt_threshold', math.pi / 3.0)
        self.target_models = self.get_parameter('target_models').value
        self.prowling_step = self.get_parameter('prowling_speed').value * UPDATE_PERIOD
        self.hit_sock_duration = Duration(seconds=self.get_parameter('hit_sock_duration').value)
        self.outside_x = self.get_parameter('outside_x').value
        self.outside_y = self.get_parameter('outside_y').value
        self.hit_tilt_threshold = self.get_parameter('hit_tilt_threshold').value
        self.alive_cats = {}  # dictionary to store relevant events for still-alive cats
        self.killed_cats = []  # casualties record

        self.set_pose_client = self.create_client(SetEntityPose, '/world/default/set_pose')
        self.create_subscription(Contacts, '/cats/contacts', self.contacts_cb, 10)
        self.odometry_sub = self.create_subscription(Odometry, '/cats/odometry', self.odometry_cb, 10)
        self.timer = self.create_timer(UPDATE_PERIOD, self.update)

    def handle_contact(self, involved1, involved2, position):
        if involved1 in self.alive_cats:
            if involved2.startswith('rocket'):
                self.alive_cats[involved1]['last_hit'] = self.get_clock().now()
            else:
                self.alive_cats[involved1]['contact'] = position

    def contacts_cb(self, msg):
        # the contacts come on every simulation step while touching, so we just keep the last one
        for contact in msg.contacts:
            involved1 = contact.collision1.name.split('::')[0]
            involved2 = contact.collision2.name.split('::')[0]
            self.handle_contact(involved1, involved2, contact.positions[0])
            self.handle_contact(involved2, involved1, contact.positions[0])

    def odometry_cb(self, msg):
        model_name = msg.child_frame_id.split('/')[0]
        if model_name in self.alive_cats:
            self.alive_cats[model_name]['pose'] = msg.pose.pose
        elif model_name not in self.killed_cats:
            # first time to see this model; keep track of it if listed under target models
            # (individual instances are numbered _0, _1 and so by the model spawner)
            if model_name in self.target_models or model_name.rsplit('_', 1)[0] in self.target_models:
                self.alive_cats[model_name] = {'pose': msg.pose.pose, 'last_hit': None, 'contact': None}

    def update(self):
        if self.killed_cats and not self.alive_cats:
            self.get_logger().info("All cats toppled; stop controlling them")
            self.timer.cancel()
            self.destroy_subscription(self.odometry_sub)
            return
        if not self.set_pose_client.service_is_ready():
            self.get_logger().warning("Waiting for Gazebo's set_pose service", throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()
        for model_name, cat in list(self.alive_cats.items()):
            pose = cat['pose']
            if tilt(pose.orientation) > self.hit_tilt_threshold:
                # we consider a cat toppled after it tilts by more than hit_tilt_threshold
                self.get_logger().info(f"Cat {model_name} toppled at {pose3d2str(pose)}! "
                                       f"(tilt > {self.hit_tilt_threshold:g})")
                # send toppled cats out of the house and stop tracking them
                self.set_pose(model_name, create_2d_pose(self.outside_x + len(self.killed_cats), self.outside_y,
                                                         math.pi / 2.0))
                self.killed_cats.append(model_name)
                del self.alive_cats[model_name]
            elif cat['last_hit'] and now - cat['last_hit'] < self.hit_sock_duration:
                # hit, so stop moving; admittedly, not the most realistic for a cat,
                # but I don't let physics work to topple the cats otherwise
                pass
            elif self.prowling_step > 0.0:
                # going around: perform small steps but changing directions whenever we touch anything
                contact = cat['contact']
                if contact:
                    # limit the possible orientations to those away from the contact point
                    away_dir = math.atan2(pose.position.y - contact.y, pose.position.x - contact.x)
                    new_yaw = random.uniform(away_dir - math.pi / 3.0, away_dir + math.pi / 3.0)
                    pose.orientation = quaternion_msg_from_yaw(new_yaw)
                    self.get_logger().debug(f"{model_name}: contact from {away_dir:.2f}; new direction {new_yaw:.2f}")
                    cat['contact'] = None
                translate_pose(pose, self.prowling_step, 'x')
                self.set_pose(model_name, pose)

    def set_pose(self, model_name, pose):
        request = SetEntityPose.Request()
        request.entity.name = model_name
        request.entity.type = Entity.MODEL
        request.pose = pose
        self.set_pose_client.call_async(request)


def main():
    rclpy.init()
    node = CatsController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
