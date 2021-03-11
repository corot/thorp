#!/usr/bin/env python

import rospy

from smach.user_data import UserData

from thorp_toolkit.geometry import TF2, create_2d_pose
from thorp_smach.states.navigation import LookToPose
from thorp_smach.utils import wait_for_mbf


if __name__ == '__main__':
    rospy.init_node('run_look_to_pose_state')

    # MBF is the last component to start, so wait for it before running the sm
    wait_for_mbf()

    ud = UserData()
    ud['robot_pose'] = TF2().transform_pose(None, 'base_footprint', 'map')
    ud['target_pose'] = create_2d_pose(0.0, 0.0, 0.0, 'map')
    state = LookToPose()
    t0 = rospy.get_time()
    outcome = state.execute(ud)
    rospy.loginfo("LookToPose completed in %.2fs with outcome '%s'", rospy.get_time() - t0, outcome)

    # Wait for ctrl-c to stop the application
    rospy.spin()

    rospy.signal_shutdown('All done.')
