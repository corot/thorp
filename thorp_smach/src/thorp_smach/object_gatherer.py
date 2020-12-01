#!/usr/bin/env python

import rospy
import smach
import smach_ros

from thorp_toolkit.geometry import TF2
from thorp_toolkit.reconfigure import Reconfigure

from toolkit.common_states import wait_for_sim_time, wait_for_mbf
from toolkit.navigation_states import GetRobotPose, LookToPose
from toolkit.perception_states import MonitorTables
from toolkit.exploration_states import ExploreHouse
from toolkit.gathering_states import GatherObjects


def object_gatherer_sm():
    """
    Object gatherer SM:
     - explore house while looking for tables
     - approach each detected table and pick all the objects of the chosen shape
    """

    # gets called when ANY child state terminates
    def child_term_cb(outcome_map):

        # terminate all running states if FOO finished with outcome 'outcome3'
        if outcome_map['EXPLORE_HOUSE']:
            return True

        # terminate all running states if BAR finished
        if outcome_map['DETECT_TABLES']:
            return True

        # in all other case, just keep running, don't terminate anything
        return False

    # gets called when ALL child states are terminated
    def out_cb(outcome_map):
        if outcome_map['DETECT_TABLES'] == 'succeeded':
            return 'detected'
        if outcome_map['EXPLORE_HOUSE'] == 'succeeded':
            return 'not_detected'
        return outcome_map['EXPLORE_HOUSE']

    # concurrence state machine: detect tables while exploring the house
    search_sm = smach.Concurrence(outcomes=['detected', 'not_detected', 'aborted', 'preempted'],
                                  default_outcome='not_detected',
                                  output_keys=['table', 'table_pose'],
                                  child_termination_cb=child_term_cb,
                                  outcome_cb=out_cb)
    with search_sm:
        smach.Concurrence.add('EXPLORE_HOUSE', ExploreHouse())
        smach.Concurrence.add('DETECT_TABLES', MonitorTables())

    # confirm detection state machine
    confirm_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                connector_outcome='succeeded',
                                input_keys=['table', 'table_pose'],
                                output_keys=['table', 'table_pose'])
    with confirm_sm:
        smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
        smach.Sequence.add('TURN_TO_TABLE', LookToPose(),
                           remapping={'target_pose': 'table_pose'})
        smach.Sequence.add('CONFIRM_TABLE', MonitorTables(2))  # 2s timeout

    # Full SM: explore the house and gather objects from all detected tables
    sm = smach.StateMachine(outcomes=['detected',
                                      'not_detected',
                                      'tray_full',
                                      'aborted',
                                      'preempted'])
    with sm:
        smach.StateMachine.add('SEARCH', search_sm,
                               transitions={'detected': 'CONFIRM',
                                            'not_detected': 'not_detected',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('CONFIRM', confirm_sm,
                               transitions={'succeeded': 'GATHER',
                                            'aborted': 'SEARCH',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('GATHER', GatherObjects(),
                               transitions={'succeeded': 'SEARCH',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted',
                                            'tray_full': 'tray_full'})
                               # transitions={'succeeded': 'SEARCH',
                               #              'aborted': 'SEARCH',
                               #              'preempted': 'SEARCH'})
    return sm


if __name__ == '__main__':
    rospy.init_node('smach_object_gatherer')

    TF2()  # start listener asap to avoid delays when running

    Reconfigure().load_named_configs()  # load named configurations from the default location

    wait_for_sim_time()

    sm = object_gatherer_sm()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # MBF is the last component to start, so wait for it before running the sm
    wait_for_mbf()

    # Execute the state machine
    t0 = rospy.get_time()
    outcome = sm.execute()
    rospy.loginfo("Gathering completed in %.2fs with outcome '%s'", rospy.get_time() - t0, outcome)

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

    rospy.signal_shutdown('All done.')
