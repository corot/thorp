#!/usr/bin/env python

import math
import rospy

from thorp_msgs.srv import CannonCmd, CannonCmdRequest
from thorp_msgs.msg import ThorpError
from arbotix_msgs.msg import Digital, Analog
from sensor_msgs.msg import JointState

rospy.init_node("cannon_ctrl")

def handle_cannon_command(request):
    if (request.action == CannonCmdRequest.TILT or CannonCmdRequest.BOTH):
        if request.angle < -5.0 or request.angle > 18.0:
            # The -5 is due to a strange effect on OpenCM Servo class; -6 puts the cannon almost in collision
            # with the upper plate!  TODO investigate what's going on
            return ThorpError(code=ThorpError.JOINT_OUT_OF_BOUNDS, text="Tilt angle out of bounds (-5,+18)")
        rospy.loginfo("Tilting cannon to %.2f degrees", request.angle)
        global cannon_tilt_angle
        cannon_tilt_angle = math.radians(request.angle)
        # OpenCM servo is configured to operate between 150 and 210, being 180 the central position
        tilt_cannon_pub.publish(Analog(value=int(request.angle + 180)))

    if (request.action == CannonCmdRequest.FIRE or CannonCmdRequest.BOTH) and request.shots > 0:
        rospy.loginfo("Firing cannon! %d shot%s", request.shots, 's' if request.shots > 1 else '')
        fire_cannon_pub.publish(Digital(value=1))
        rospy.sleep(rospy.Duration(request.shots * 0.055))
        fire_cannon_pub.publish(Digital(value=0))

    return ThorpError(code=ThorpError.SUCCESS)


cannon_cmd_srv = rospy.Service('cannon_command', CannonCmd, handle_cannon_command)

tilt_cannon_pub = rospy.Publisher('arbotix/cannon_servo', Analog, queue_size=5, latch=True)
fire_cannon_pub = rospy.Publisher('arbotix/cannon_trigger', Digital, queue_size=5, latch=True)

# Publish cannon joint state
cannon_tilt_angle = 0.0
joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=5)

msg = JointState()
msg.name = ["cannon_joint"]

while not rospy.is_shutdown():
    msg.position = [cannon_tilt_angle]
    msg.velocity = [0.0]
    msg.header.stamp = rospy.Time.now()
    joint_states_pub.publish(msg)
    rospy.sleep(0.05)
