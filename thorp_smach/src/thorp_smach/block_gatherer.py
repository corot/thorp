#!/usr/bin/env python

import rospy
import smach
import smach_ros

from thorp_smach.toolkit.gathering_states import PickReachableObjs
from thorp_smach.toolkit.comon_states import wait_for_sim_time, wait_for_mbf


def main():
    rospy.init_node('smach_block_gatherer')

    if not wait_for_sim_time():
        return

    sm = smach.StateMachine(outcomes=['succeeded',
                                      'aborted',
                                      'preempted',
                                      'tray_full'])
    with sm:
        smach.StateMachine.add('GATHER_BLOCKS', PickReachableObjs())

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # MBF is the last component to start, so wait for it before running the sm
    wait_for_mbf()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()
