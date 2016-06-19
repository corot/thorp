#!/usr/bin/env python

import math
import rospy

from thorp_msgs.srv import CannonCmd, CannonCmdRequest
from thorp_msgs.msg import ThorpError
from arbotix_msgs.msg import Analog
from kobuki_msgs.msg import ExternalPower, RobotStateEvent
from sensor_msgs.msg import JointState

rospy.init_node("cannon_ctrl")

def handle_robot_state_event(event):
    if event.state == RobotStateEvent.ONLINE:
        fire_cannon_pub.publish(ExternalPower(state=ExternalPower.OFF, source=ExternalPower.PWR_3_3V1A))
        tilt_cannon_pub.publish(Analog(value=0))

def handle_cannon_command(request):
    if request.action == CannonCmdRequest.TILT or CannonCmdRequest.BOTH:
        if request.angle < -30.0 or request.angle > 30.0:
            return ThorpError(code=ThorpError.JOINT_OUT_OF_BOUNDS, text="Tilt angle out of bounds (-30,+30)")
        cannon_tilt_angle = math.radians(request.angle)
        # OpenCM servos operate between 0 and 180, being 90 the central position
        tilt_cannon_pub.publish(Analog(value=int(request.angle) + 90))

    if request.action == CannonCmdRequest.FIRE or CannonCmdRequest.BOTH:
        fire_cannon_pub.publish(ExternalPower(state=ExternalPower.ON, source=ExternalPower.PWR_3_3V1A))
        rospy.sleep(rospy.Duration(request.shots * 0.05))
        fire_cannon_pub.publish(ExternalPower(state=ExternalPower.OFF, source=ExternalPower.PWR_3_3V1A))

    return ThorpError(code=ThorpError.SUCCESS)


cannon_cmd_srv = rospy.Service('cannon_command', CannonCmd, handle_cannon_command)

tilt_cannon_pub = rospy.Publisher('arbotix/cannon_servo', Analog, queue_size=5, latch=True)
fire_cannon_pub = rospy.Publisher('mobile_base/commands/external_power', ExternalPower, queue_size=5, latch=True)

robot_state_sub = rospy.Subscriber('mobile_base/events/robot_state', RobotStateEvent, handle_robot_state_event)

# Switch off both servo and external power immediately at startup and whenever robot serial port goes back online
handle_robot_state_event(RobotStateEvent(state=RobotStateEvent.ONLINE))

# Publish cannon joint state
cannon_tilt_angle = 0.0
joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=5)

msg = JointState()
msg.name = ["cannon_joint"]

while not rospy.is_shutdown():
    msg.position = [cannon_tilt_angle]
    msg.velocity = [0.0]  # TODO (if possible)
    msg.header.stamp = rospy.Time.now()
    joint_states_pub.publish(msg)
    rospy.sleep(0.05)
