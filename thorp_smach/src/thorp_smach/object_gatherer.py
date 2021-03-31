#!/usr/bin/env python

import rospy
import smach

from states.navigation import GetRobotPose, LookToPose
from states.perception import MonitorTables, TableMarkVisited, TableWasVisited, CheckTableSize
from states.manipulation import FoldArm
from states.exploration import ExploreHouse
from states.gathering import GatherObjects
from states.common import Sleep
from containers.do_on_exit import DoOnExit as DoOnExitContainer

from utils import run_sm


def object_gatherer_sm(target_types):
    """
    Object gatherer SM:
     - explore house while looking for tables
     - approach each detected table and pick all the objects of the chosen types
    """

    # look for tables state machine
    detect_tables_sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'],
                                          output_keys=['table', 'table_pose'])
    with detect_tables_sm:
        smach.StateMachine.add('DETECT_TABLES', MonitorTables(),
                               transitions={'succeeded': 'VISITED_TABLE?',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('VISITED_TABLE?', TableWasVisited(),
                               transitions={'true': 'PAUSE_DETECTION',
                                            'false': 'succeeded'})
        smach.StateMachine.add('PAUSE_DETECTION', Sleep(2),  # improbable to see a new table for a while
                               transitions={'succeeded': 'DETECT_TABLES',
                                            'aborted': 'aborted'})

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
        smach.Concurrence.add('DETECT_TABLES', detect_tables_sm)

    # confirm detection state machine
    confirm_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                connector_outcome='succeeded',
                                input_keys=['table', 'table_pose'],
                                output_keys=['table', 'table_pose'])
    with confirm_sm:
        smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
        smach.Sequence.add('TURN_TO_TABLE', LookToPose(),
                           remapping={'target_pose': 'table_pose'})
        smach.Sequence.add('CONFIRM_TABLE', MonitorTables(2.0))  # 2s timeout
        smach.Sequence.add('MARK_VISITED', TableMarkVisited())
        smach.Sequence.add('VALIDATE_SIZE', CheckTableSize())

    # Full SM: explore the house and gather objects from all detected tables
    sm = DoOnExitContainer(outcomes=['detected',
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
        smach.StateMachine.add('GATHER', GatherObjects(target_types),
                               transitions={'succeeded': 'SEARCH',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted',
                                            'tray_full': 'tray_full'})
        DoOnExitContainer.add_finally('FOLD_ARM', FoldArm())  # fold arm on exit regardless of the outcome
    return sm


if __name__ == '__main__':
    rospy.init_node('object_gatherer_smach')

    target_types = rospy.get_param('~object_types', '').split()
    run_sm(object_gatherer_sm(target_types), rospy.get_param('~app_name'))
