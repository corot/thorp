#!/usr/bin/env python

"""
Fake arbotix servos services meaningless on simulation: relax, power on/off
"""
import rospy

from arbotix_msgs.srv import Relax, RelaxResponse, Enable, EnableResponse


def enable_all_cb(req):
    rospy.logdebug("Arm servos " + ('disabled', 'enabled')[req.enable])
    return EnableResponse(req.enable)


def relax_all_cb(req):
    rospy.logdebug("Arm servos relaxed")
    return RelaxResponse()


rospy.init_node("fake_servos_srv")

rospy.Service('servos/enable_all', Enable, enable_all_cb)
rospy.Service('servos/relax_all', Relax, relax_all_cb)

rospy.spin()
