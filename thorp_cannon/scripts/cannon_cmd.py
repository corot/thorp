#!/usr/bin/env python

import sys
import rospy

from thorp_msgs.srv import CannonCmd, CannonCmdRequest


def cannon_command(cmd, arg):
    rospy.wait_for_service('cannon_command')
    try:
        srv = rospy.ServiceProxy('cannon_command', CannonCmd)
        resp1 = srv(cmd, arg, arg)
        print resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def usage():
    return "%s <cmd> <arg>" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 3:
        cmd = int(sys.argv[1])
        arg = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)

    rospy.init_node('cannon_command')
    print "Requesting %d %d" % (cmd, arg)
    cannon_command(cmd, arg)
