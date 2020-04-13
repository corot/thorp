#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

rospy.init_node("fake_joint_pub")
p = rospy.Publisher('joint_states', JointState, queue_size=5)

msg = JointState()
msg.name = ["wheel_left_joint", "wheel_right_joint"]
msg.position = [0.0 for name in msg.name]
msg.velocity = [0.0 for name in msg.name]

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    p.publish(msg)
    rospy.sleep(0.05)
