#!/usr/bin/env python

import rospy

from math import atan, degrees, radians
from std_msgs.msg import Float64
from thorp_msgs.srv import CannonCmd, CannonCmdRequest
from thorp_msgs.msg import ThorpError
from arbotix_msgs.msg import Digital, Analog
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from thorp_toolkit.geometry import transform_pose, heading


class CannonCtrlNode:
    def __init__(self):
        self._simulation = rospy.get_param('/use_sim_time', False)

        self._cannon_cmd_srv = rospy.Service('cannon_command', CannonCmd, self.handle_cannon_command)

        self._fire_cannon_pub = rospy.Publisher('arbotix/cannon_trigger', Digital, queue_size=5, latch=True)
        if self._simulation:
            self._tilt_cannon_pub = rospy.Publisher('cannon_joint/command', Float64, queue_size=5, latch=True)
        else:
            self._tilt_cannon_pub = rospy.Publisher('arbotix/cannon_servo', Analog, queue_size=5, latch=True)

        # Subscribe to a target pose to aim to
        self._target_obj_pose = None
        self._target_obj_sub = rospy.Subscriber('target_object_pose', PoseStamped, self.target_obj_cb, queue_size=5)

        # Publish cannon joint state
        self._cannon_tilt_angle = 0.0
        self._joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=5)

        self._js_msg = JointState()
        self._js_msg.name = ["cannon_joint"]
        self._js_msg.velocity = [0.0]

    def target_obj_cb(self, pose):
        self._target_obj_pose = pose

    def aim_to_target(self):
        if not self._target_obj_pose:
            return ThorpError(code=ThorpError.INVALID_TARGET_POSE, text="No target pose to aim to")
        pose_in_cannon_ref = transform_pose('cannon_link', self._target_obj_pose)
        adjacent = pose_in_cannon_ref.pose.position.x
        opposite = pose_in_cannon_ref.pose.position.z
        tilt_angle = atan(opposite / adjacent)
        print(degrees(tilt_angle))
        return self.tilt(degrees(tilt_angle))

    def tilt(self, angle):
        if abs(angle) > 18.0:
            return ThorpError(code=ThorpError.JOINT_OUT_OF_BOUNDS, text="Tilt angle out of bounds (-18, +18)")
        rospy.loginfo("Tilting cannon to %.2f degrees", angle)
        if angle < 0.0 and not self._simulation:
            # With real cannon I need to squeeze negative angles between 0 and -5, due to a strange effect on OpenCM
            # Servo class; -6 puts the cannon almost in collision with the upper plate! TODO investigate what's going on
            angle = (angle * 5.0) / 18.0

        self._cannon_tilt_angle = radians(angle)
        if self._simulation:
            self._tilt_cannon_pub.publish(Float64(data=self._cannon_tilt_angle))
        else:
            # OpenCM servo is configured to operate between 150 and 210, being 180 the central position
            self._tilt_cannon_pub.publish(Analog(value=int(round(angle + 180))))
        return ThorpError(code=ThorpError.SUCCESS)

    def fire(self, shots):
        if shots > 0:
            rospy.loginfo("Firing cannon! %d shot%s", shots, 's' if shots > 1 else '')
            msg = Digital(value=1)
            msg.header.stamp = rospy.get_rostime()
            self._fire_cannon_pub.publish(msg)
            rospy.sleep(rospy.Duration(shots * 0.055))
            msg.value = 0
            msg.header.stamp = rospy.get_rostime()
            self._fire_cannon_pub.publish(msg)
        return ThorpError(code=ThorpError.SUCCESS)

    def handle_cannon_command(self, request):
        if request.action == CannonCmdRequest.AIM:
            return self.aim_to_target()

        if request.action == CannonCmdRequest.TILT:
            return self.tilt(request.angle)

        if request.action == CannonCmdRequest.FIRE:
            return self.fire(request.shots)

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._js_msg.position = [self._cannon_tilt_angle]
            self._js_msg.header.stamp = rospy.Time.now()
            self._joint_states_pub.publish(self._js_msg)
            try:
                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                pass


if __name__ == '__main__':
    rospy.init_node("cannon_ctrl")

    node = CannonCtrlNode()
    node.spin()
