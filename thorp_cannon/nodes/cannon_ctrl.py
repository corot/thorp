#!/usr/bin/env python3

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from math import atan, degrees, radians
from std_msgs.msg import Bool, Float64MultiArray, UInt16
from thorp_msgs.srv import CannonCommand
from thorp_msgs.msg import ThorpError
from geometry_msgs.msg import PoseStamped
from thorp_toolkit.common import init
from thorp_toolkit.geometry import TF2


class CannonCtrlNode(Node):
    def __init__(self):
        super().__init__('cannon_ctrl')
        init(self)

        self._simulation = self.get_parameter('use_sim_time').value
        if not self._simulation:
            # TODO: the real cannon is commanded through the arbotix board, not ported yet
            raise RuntimeError("Real cannon not supported yet; only simulation")

        # the service sleeps while firing, so it must not block the clock subscription
        self._cannon_cmd_srv = self.create_service(CannonCommand, 'cannon_command', self.handle_cannon_command,
                                                   callback_group=ReentrantCallbackGroup())

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._tilt_cannon_pub = self.create_publisher(Float64MultiArray, 'cannon_joint_controller/commands', 5)
        self._shots_left = 1000
        self._fire_cannon_pub = self.create_publisher(Bool, 'arbotix/cannon_trigger', latched)
        self._shots_left_pub = self.create_publisher(UInt16, '~/shots_left', latched)
        self._shots_left_pub.publish(UInt16(data=self._shots_left))

        # Subscribe to a target pose to aim to
        self._target_obj_pose = None
        self._target_obj_sub = self.create_subscription(PoseStamped, 'target_object_pose', self.target_obj_cb, 5)

    def target_obj_cb(self, pose):
        self._target_obj_pose = pose

    def aim_to_target(self):
        if not self._target_obj_pose:
            self.get_logger().warning("No target pose to aim to")
            return ThorpError(code=ThorpError.INVALID_TARGET_POSE, text="No target pose to aim to")
        pose_in_cannon_ref = TF2().transform_pose(self._target_obj_pose, None, 'cannon_shaft_link')
        adjacent = pose_in_cannon_ref.pose.position.x
        opposite = pose_in_cannon_ref.pose.position.z
        tilt_angle = atan(opposite / adjacent)
        print(adjacent, opposite, degrees(tilt_angle))
        tilt_angle = max(min(degrees(tilt_angle), +18), -18)
        return self.tilt(tilt_angle)

    def tilt(self, angle):
        if abs(angle) > 18.0:
            err_msg = "Tilt angle %g out of bounds (-18, +18)" % angle
            self.get_logger().warning(err_msg)
            return ThorpError(code=ThorpError.JOINT_OUT_OF_BOUNDS, text=err_msg)
        self.get_logger().info("Tilting cannon to %g degrees" % angle)
        self._tilt_cannon_pub.publish(Float64MultiArray(data=[radians(angle)]))
        return ThorpError(code=ThorpError.SUCCESS)

    def fire(self, shots):
        if shots > 0:
            self._fire_cannon_pub.publish(Bool(data=True))
            self.get_clock().sleep_for(Duration(seconds=shots * 0.055))
            self._fire_cannon_pub.publish(Bool(data=False))
            self._shots_left -= shots
            self._shots_left_pub.publish(UInt16(data=self._shots_left))
            self.get_logger().info("Firing %d cannon shot%s! (%d left)"
                                   % (shots, 's' if shots > 1 else '', self._shots_left))
        return ThorpError(code=ThorpError.SUCCESS)

    def handle_cannon_command(self, request, response):
        if request.action == CannonCommand.Request.AIM:
            response.error = self.aim_to_target()
        elif request.action == CannonCommand.Request.TILT:
            response.error = self.tilt(request.angle)
        elif request.action == CannonCommand.Request.FIRE:
            response.error = self.fire(request.shots)
        return response


def main():
    rclpy.init()
    node = CannonCtrlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
