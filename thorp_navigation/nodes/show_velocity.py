#!/usr/bin/env python3

"""
Show current velocity and travelled distance on RViz at top-left corner, as an overlay text.
We also republish current velocity as a TwistStamped and the linear as a Float32 msg

Author:
    Jorge Santos
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from rviz_2d_overlay_msgs.msg import OverlayText

from thorp_toolkit.common import init
from thorp_toolkit.geometry import TF2
from thorp_toolkit.tachometer import Tachometer
from thorp_toolkit.visualization import Visualization


def get_robot_pose(target_frame):
    try:
        return TF2().transform_pose(None, 'base_footprint', target_frame)
    except RuntimeError:
        return None  # not localized yet


class ShowVelocity(Node):
    def __init__(self):
        super().__init__('show_velocity')
        init(self)
        self.tachometer = Tachometer(get_robot_pose, 'map')
        self.tachometer.start()
        self.speed_pub = self.create_publisher(Float32, 'rviz/linear_speed', 1)
        self.twist_pub = self.create_publisher(TwistStamped, 'rviz/twist_stamped', 1)
        self.millage_pub = self.create_publisher(OverlayText, 'rviz/millage_overlay', 1)
        self.twist = TwistStamped()
        self.twist.header.frame_id = 'base_footprint'
        self.create_timer(0.1, self.publish)

    def publish(self):
        if self.tachometer.odometry:
            self.twist.header.stamp = self.get_clock().now().to_msg()
            self.twist.twist = self.tachometer.odometry.twist.twist
            self.twist_pub.publish(self.twist)
            self.speed_pub.publish(Float32(data=self.twist.twist.linear.x))
            millage = f"v: {round(self.twist.twist.linear.x, 2)} m/s\t \
                        w: {round(self.twist.twist.angular.z, 2)} rad/s\t \
                        d: {round(self.tachometer.distance, 1)} m"
            self.millage_pub.publish(Visualization.create_overlay_text(40, (0.1, 1.0, 0.9), millage, 12))


def main():
    rclpy.init()
    node = ShowVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tachometer.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
