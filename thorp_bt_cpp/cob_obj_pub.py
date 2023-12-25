#!/usr/bin/env python

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped

from rosgraph_msgs.msg import Clock
from cob_perception_msgs.msg import Detection, DetectionArray


def nav_goal_cb(msg):

    rospy.sleep(0.5)
    d = Detection(label='cat', pose=msg)
    msg = DetectionArray()
    msg.detections.append(d)
    p.publish(msg)


rospy.init_node("fake_joint_pub")

cp = rospy.Publisher('clock', Clock, queue_size=5)
p = rospy.Publisher('tracked_objects', DetectionArray, queue_size=5)

rospy.Subscriber('/move_base_simple/goal', PoseStamped, nav_goal_cb)

rospy.spin()

