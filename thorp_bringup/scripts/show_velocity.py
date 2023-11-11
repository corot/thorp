#!/usr/bin/env python

"""
Show current velocity and travelled distance on RViz at top-left corner, using jsk_rviz_plugins.
We also republish current velocity as a TwistStamped and the linear as a Float32 msg
Author:
    Jorge Santos
"""

import rospy

from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from jsk_rviz_plugins.msg import OverlayText

from thorp_toolkit.geometry import TF2
from thorp_toolkit.tachometer import Tachometer
from thorp_toolkit.visualization import Visualization


def get_robot_pose(target_frame):
    return TF2().transform_pose(None, 'base_footprint', target_frame)


if __name__ == "__main__":
    rospy.init_node("show_velocity")

    TF2()  # start listener asap and wait till buffer fill  TODO inside TF2
    rospy.sleep(1)

    tachometer = Tachometer(get_robot_pose, 'map')
    tachometer.start()
    start_time = rospy.get_time()

    speed_pub = rospy.Publisher('rviz/linear_speed', Float32, queue_size=1)
    twist_pub = rospy.Publisher('rviz/twist_stamped', TwistStamped, queue_size=1)
    millage_pub = rospy.Publisher('rviz/millage_overlay', OverlayText, queue_size=1)

    rate = rospy.Rate(10)
    twist = TwistStamped()
    twist.header.frame_id = 'base_footprint'
    while not rospy.is_shutdown():
        if tachometer.odometry:
            twist.header.stamp = rospy.get_rostime()
            twist.twist = tachometer.odometry.twist.twist
            twist_pub.publish(twist)
            speed_pub.publish(Float32(twist.twist.linear.x))

            millage = f"v: {round(twist.twist.linear.x, 2)} m/s\t \
                        w: {round(twist.twist.angular.z, 2)} rad/s\t \
                        d: {round(tachometer.distance, 1)} m"
            millage_pub.publish(Visualization.create_overlay_text(40, (0.1, 1.0, 0.9), millage, 12))
        rate.sleep()
