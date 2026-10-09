#!/usr/bin/env python3
"""
Fix the objects resting on Thorp's tray to Thorp in Gazebo, by adding a DetachableJoint system to Thorp's model for
each of them. Otherwise, objects on the tray join Thorp's whole body in DART's contact solving, slowing the simulation
more with every object. An object rests on the tray when it's within the tray's footprint, just over its surface, with
the gripper away and still for a while.
"""

import math
import threading

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

ROBOT_MODEL = 'thorp'
# tray_link is merged into base_footprint in Gazebo, as joined to it by fixed joints
PARENT_LINK = 'base_footprint'
CHILD_LINK = 'link'  # the only link of Thorp's object models

MAX_HEIGHT_OVER_TRAY = 0.06  # object centers over the tray surface, as the tallest objects are 5 cm
FOOTPRINT_MARGIN = 0.01  # objects can slightly overhang the tray's slots
MIN_GRIPPER_DISTANCE = 0.06  # the gripper has released the object and moved away
STILL_TIME = 0.5  # seconds the object must stay still on the tray
STILL_TOLERANCE = 0.002  # meters it can move meanwhile


def to_robot_frame(point, robot):
    """ A Gazebo world point on the robot frame, given the robot's pose as (x, y, z, qx, qy, qz, qw) """
    x, y, z = point[0] - robot[0], point[1] - robot[1], point[2] - robot[2]
    qx, qy, qz, qw = robot[3:]
    # rotate by the inverse rotation
    qx, qy, qz = -qx, -qy, -qz
    tx, ty, tz = 2.0 * (qy * z - qz * y), 2.0 * (qz * x - qx * z), 2.0 * (qx * y - qy * x)
    return (x + qw * tx + qy * tz - qz * ty, y + qw * ty + qz * tx - qx * tz, z + qw * tz + qx * ty - qy * tx)


class TrayObjects(Node):

    def __init__(self):
        super().__init__('tray_objects')
        self.world = self.declare_parameter('world', 'default').value
        self.tray_link = self.declare_parameter('tray.link', 'tray_link').value
        self.tray_half_x = self.declare_parameter('tray.side_x', 0.14).value / 2.0 + FOOTPRINT_MARGIN
        self.tray_half_y = self.declare_parameter('tray.side_y', 0.14).value / 2.0 + FOOTPRINT_MARGIN

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tray = None
        self.lock = threading.Lock()
        self.poses = {}  # model name: (id, (x, y, z, qx, qy, qz, qw)) on the world frame
        self.candidates = {}  # model name: (time first seen still on the tray, position on the robot frame)
        self.fixed = set()

        self.gz_node = GzNode()
        topic = f'/world/{self.world}/pose/info'
        if not self.gz_node.subscribe(Pose_V, topic, self.poses_cb):
            raise RuntimeError(f'Subscribing to {topic} failed')
        self.create_timer(0.2, self.check)
        self.get_logger().info('Fixing the objects resting on the tray to Thorp')

    def poses_cb(self, msg):
        poses = {p.name: (p.id, (p.position.x, p.position.y, p.position.z,
                                 p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                 for p in msg.pose}
        with self.lock:
            self.poses = poses

    def lookup(self, frame):
        tf = self.tf_buffer.lookup_transform('base_footprint', frame, Time(), Duration(seconds=0.1))
        return tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z

    def check(self):
        with self.lock:
            poses = dict(self.poses)
        if ROBOT_MODEL not in poses:
            return
        try:
            if self.tray is None:
                self.tray = self.lookup(self.tray_link)
            gripper = self.lookup('gripper_link')
        except TransformException:
            return
        robot_id, robot = poses[ROBOT_MODEL]
        now = self.get_clock().now()
        for name, (_, pose) in poses.items():
            if name == ROBOT_MODEL or name in self.fixed:
                continue
            position = to_robot_frame(pose, robot)
            dx, dy, dz = (position[i] - self.tray[i] for i in range(3))
            on_tray = (abs(dx) <= self.tray_half_x and abs(dy) <= self.tray_half_y and
                       0.0 <= dz <= MAX_HEIGHT_OVER_TRAY and math.dist(position, gripper) >= MIN_GRIPPER_DISTANCE)
            if not on_tray:
                self.candidates.pop(name, None)
                continue
            since, first = self.candidates.get(name, (now, position))
            if math.dist(position, first) > STILL_TOLERANCE:
                since, first = now, position
            self.candidates[name] = (since, first)
            if (now - since).nanoseconds * 1e-9 >= STILL_TIME:
                self.fix(robot_id, name)

    def fix(self, robot_id, name):
        """ Add a DetachableJoint system to Thorp's model, fixing the object to it where it is """
        request = EntityPlugin_V()
        request.entity.id = robot_id
        request.entity.type = Entity.MODEL
        plugin = request.plugins.add()
        plugin.name = 'gz::sim::systems::DetachableJoint'
        plugin.filename = 'gz-sim-detachable-joint-system'
        # each system needs its own topics, as Thorp's model gets one per object
        topic = '/thorp/tray/' + ''.join(c if c.isalnum() else '_' for c in name)
        plugin.innerxml = (f'<parent_link>{PARENT_LINK}</parent_link><child_model>{name}</child_model>'
                           f'<child_link>{CHILD_LINK}</child_link><detach_topic>{topic}/detach</detach_topic>'
                           f'<attach_topic>{topic}/attach</attach_topic><output_topic>{topic}/state</output_topic>')
        service = f'/world/{self.world}/entity/system/add'
        ok, response = self.gz_node.request(service, request, EntityPlugin_V, Boolean, 1000)
        if ok and response.data:
            self.fixed.add(name)
            self.candidates.pop(name, None)
            self.get_logger().info(f'{name} fixed to the tray')
        else:
            self.get_logger().error(f'Fixing {name} to the tray failed')


def main():
    rclpy.init()
    node = TrayObjects()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
