#!/usr/bin/env python3

"""
Publish a fixed state for the gripper joints no servo or simulator provides: gripper_link_joint, only used as
the tool frame, and the static finger joint.

Port of turtlebot_arm_bringup's fake_joint_pub.py, from https://github.com/corot/turtlebot_arm, thorp branch.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeJointPub(Node):

    def __init__(self):
        super().__init__('fake_joint_pub')
        self.pub = self.create_publisher(JointState, 'joint_states', 5)

        self.msg = JointState()
        self.msg.name = ['gripper_link_joint', 'gripper_static_joint']
        self.msg.position = [0.0] * len(self.msg.name)
        self.msg.velocity = [0.0] * len(self.msg.name)

        self.create_timer(0.2, self.publish)

    def publish(self):
        # make the state valid into the future to allow slow rate
        self.msg.header.stamp = (self.get_clock().now() + Duration(seconds=0.5)).to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = FakeJointPub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
