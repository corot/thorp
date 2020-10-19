#!/usr/bin/env python

import rospy
import smach
import smach_ros

from turtlebot_arm_block_manipulation.msg import *
from thorp_smach.toolkit.manipulation_states import GatherBlocks
from thorp_smach.toolkit.comon_states import wait_for_sim_time


def main():
    rospy.init_node('smach_block_gatherer')

    if not wait_for_sim_time():
        return

    sm = smach.StateMachine(outcomes=['succeeded',
                                      'aborted',
                                      'preempted',
                                      'tray_full'])
    with sm:
        # app config
        sm.userdata.frame = rospy.get_param('~arm_link', 'arm_base_link')
        sm.userdata.table_height = rospy.get_param('~table_height', -0.03)
        sm.userdata.block_size = rospy.get_param('~block_size', 0.025)

        def result_cb(ud, status, result):
            ud['blocks'] = result.blocks
            ud['object_names'] = ['block' + str(i) for i in range(1, len(result.blocks.poses) + 1)]

        smach.StateMachine.add('BLOCK_DETECTION',
                               smach_ros.SimpleActionState('block_detection',
                                                           BlockDetectionAction,
                                                           goal_slots=['frame', 'table_height', 'block_size'],
                                                           result_slots=['blocks'],
                                                           result_cb=result_cb,
                                                           output_keys=['blocks', 'object_names']),
                               transitions={'succeeded': 'GATHER_BLOCKS',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('GATHER_BLOCKS', GatherBlocks(),
                               remapping={'blocks': 'objects'},
                               transitions={'succeeded': 'succeeded',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted',
                                            'tray_full': 'tray_full'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()
