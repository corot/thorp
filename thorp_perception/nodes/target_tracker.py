#!/usr/bin/env python3

"""
Target tracker: picks the object to hunt among YOLO's 3D detections of the target classes, and publishes its pose on
the fixed frame, for following and aiming at it. It keeps the same target while it's detected, or else picks the
nearest one. Detections must be on a robot frame, as distances are measured from its origin.
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from yolo_msgs.msg import DetectionArray

from thorp_toolkit.common import init
from thorp_toolkit.geometry import TF2, point2d2str


class TargetTracker(Node):
    def __init__(self):
        super().__init__('target_tracker')
        init(self)
        self.declare_parameter('target_classes', ['cat', 'dog', 'horse'])
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('fixed_frame', 'odom')
        self.target_classes = set(self.get_parameter('target_classes').value)
        self.min_score = self.get_parameter('min_score').value
        self.fixed_frame = self.get_parameter('fixed_frame').value

        self.target_id = None
        self.target_pub = self.create_publisher(PoseStamped, 'target_object_pose', 1)
        self.create_subscription(DetectionArray, 'yolo/detections_3d', self.detections_callback, 1)
        self.get_logger().info(f"Tracking {', '.join(sorted(self.target_classes))}")

    def detections_callback(self, msg):
        candidates = [d for d in msg.detections if d.class_name in self.target_classes and d.score >= self.min_score]
        if not candidates:
            return
        # without tracking, detections have no id, and each frame picks the nearest
        target = next((d for d in candidates if d.id and d.id == self.target_id), None)
        if target is None:
            target = min(candidates, key=lambda d: math.hypot(d.bbox3d.center.position.x, d.bbox3d.center.position.y))
            self.target_id = target.id
            position = target.bbox3d.center.position
            self.get_logger().info(f"Target: {target.class_name} {target.id} ({target.score:.2f}) at "
                                   f"{math.hypot(position.x, position.y):.2f} m, {point2d2str(position)}")

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = target.bbox3d.frame_id
        pose.pose = target.bbox3d.center
        try:
            # on the image's time, as the robot moves while detecting
            self.target_pub.publish(TF2().transform_pose(pose, None, self.fixed_frame, Duration(seconds=0.2)))
        except RuntimeError as e:
            self.get_logger().warning(str(e), throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = TargetTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
