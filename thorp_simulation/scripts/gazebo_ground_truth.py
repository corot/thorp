#!/usr/bin/env python

"""
Use Gazebo ground truth for perfect localization
"""

import rospy

from std_msgs.msg import Header
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped

from thorp_toolkit.geometry import TF2
from thorp_toolkit.transform import Transform


def link_states_cb(msg):
    try:
        # throttle to 'frequency' Hz
        if hasattr(link_states_cb, 'last_pub_time') and rospy.get_time() - link_states_cb.last_pub_time < period:
            return
        link_states_cb.last_pub_time = rospy.get_time()

        # get map -> base from gazebo
        index = msg.name.index('thorp::base_footprint')
        map_to_bfp_tf = Transform.create(PoseStamped(Header(stamp=rospy.get_rostime(), frame_id='map'),
                                                     msg.pose[index]))
        # subtract (multiply by the inverse) odom -> base_footprint tf
        bfp_to_odom_tf = Transform.create(TF2().lookup_transform('base_footprint', 'odom'))
        map_to_odom_tf = map_to_bfp_tf * bfp_to_odom_tf

        TF2().publish_transform(map_to_odom_tf.to_geometry_msg_transform_stamped())
    except (ValueError, rospy.ROSException):
        pass


if __name__ == "__main__":
    rospy.init_node("gazebo_ground_truth")
    period = 1.0 / rospy.get_param("frequency", 20.0)
    rospy.Subscriber("gazebo/link_states", LinkStates, link_states_cb)
    rospy.spin()
