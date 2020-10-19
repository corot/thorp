#!/usr/bin/env python

import rospy
import smach
import smach_ros

import toolkit.config as cfg
import visualization_msgs.msg as viz_msgs

from math import pi
from thorp_toolkit.geometry import TF2, quaternion_msg_from_rpy
from turtlebot_arm_block_manipulation.msg import BlockDetectionAction
from explore_house import explore_house_sm
from toolkit.comon_states import *
from toolkit.navigation_states import *
from toolkit.exploration_states import *
from toolkit.manipulation_states import *


def gather_objects_sm():
    """
    Object gatherer SM:
     - explore house while looking for tables
     - approach each detected table and pick all the objects of the chosen shape
    """

    # explore a single room
    approach_table_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                       connector_outcome='succeeded',
                                       input_keys=['detected_table', 'detected_table_pose'],
                                       output_keys=['picking_poses', 'closest_picking_pose'])
    with approach_table_sm:
        smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
        smach.Sequence.add('CALC_PICK_POSES', CalcPickPoses(cfg.PICKING_DIST_TO_TABLE),
                           transitions={'no_valid_table': 'aborted'})
        smach.Sequence.add('APPROACH_TABLE', GoToPose(dist_tolerance=cfg.TIGHT_DIST_TOLERANCE,     # we try to
                                                      angle_tolerance=cfg.TIGHT_ANGLE_TOLERANCE),  # be precise
                           transitions={'aborted': 'aborted',
                                        'preempted': 'preempted'},
                           remapping={'target_pose': 'closest_picking_pose'})
        # TODO: ALIGN_TO_TABLE

    # explore a single room
    pick_objects_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                     connector_outcome='succeeded',
                                     input_keys=['detected_table', 'picking_poses'])
    pick_objects_sm.userdata.frame = rospy.get_param('~arm_link', 'arm_base_link')
    pick_objects_sm.userdata.table_height = rospy.get_param('~table_height', -0.03)
    pick_objects_sm.userdata.block_size = rospy.get_param('~block_size', 0.025)

    with pick_objects_sm:  # TODO iterate until there are no objects left
        smach.Sequence.add('DETECT_OBJECTS',  # TODO: will replace with a proper object detection once I have it
                           smach_ros.SimpleActionState('block_detection',
                                                       BlockDetectionAction,
                                                       goal_slots=['frame', 'table_height', 'block_size'],
                                                       result_slots=['blocks']),
                           transitions={'aborted': 'aborted',
                                        'preempted': 'preempted'},
                           remapping={'blocks': 'detected_objects'})
        smach.Sequence.add('GROUP_OBJECTS', GroupObjects(cfg.MAX_ARM_REACH, pick_objects_sm.userdata.frame),
                           transitions={'no_objects': 'aborted'},
                           remapping={'objects': 'detected_objects',
                                      'table': 'detected_table'})

    # Full SM: plan rooms visit sequence and explore each room in turn
    sm = smach.StateMachine(outcomes=['succeeded',
                                      'aborted',
                                      'preempted'],
                            input_keys=['detected_table', 'detected_table_pose'])
    with sm:
        smach.StateMachine.add('APPROACH_TABLE', approach_table_sm,
                               transitions={'succeeded': 'PICK_OBJECTS',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('PICK_OBJECTS', pick_objects_sm,
                               transitions={'succeeded': 'succeeded',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
    return sm


def object_gatherer_sm():
    """
    Object gatherer SM:
     - explore house while looking for tables
     - approach each detected table and pick all the objects of the chosen shape
    """

    # gets called when ANY child state terminates
    def child_term_cb(outcome_map):

        # terminate all running states if FOO finished with outcome 'outcome3'
        # if outcome_map['EXPLORE_HOUSE']:
        #     return True

        # terminate all running states if BAR finished
        if outcome_map['DETECT_TABLES']:
            return True

        # in all other case, just keep running, don't terminate anything
        return False

    # gets called when ALL child states are terminated
    def out_cb(outcome_map):
        if outcome_map['DETECT_TABLES'] == 'detected':
            return 'detected'
        # if outcome_map['EXPLORE_HOUSE'] == 'succeeded':
        #     return 'not_detected'
        #return outcome_map['EXPLORE_HOUSE']

    # creating the concurrence state machine
    search_sm = smach.Concurrence(outcomes=['detected', 'not_detected', 'aborted', 'preempted'],
                                  default_outcome='not_detected',
                                  output_keys=['detected_table', 'detected_table_pose'],
                                  child_termination_cb=child_term_cb,
                                  outcome_cb=out_cb)
    with search_sm:
       # smach.Concurrence.add('EXPLORE_HOUSE', explore_house_sm())
        smach.Concurrence.add('DETECT_TABLES', MonitorTables())

    # Full SM: plan rooms visit sequence and explore each room in turn
    sm = smach.StateMachine(outcomes=['detected',
                                      'not_detected',
                                      'aborted',
                                      'preempted'])
    with sm:
        smach.StateMachine.add('SEARCH', search_sm,
                               transitions={'detected': 'GATHER',
                                            'not_detected': 'not_detected',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('GATHER', gather_objects_sm(),
                               transitions={'succeeded': 'SEARCH',
                                            'aborted': 'SEARCH',
                                            'preempted': 'SEARCH'})
    return sm


if __name__ == '__main__':
    rospy.init_node('smach_object_gatherer')

    TF2()  # start listener asap to avoid delays when running

    wait_for_sim_time()

    sm = object_gatherer_sm()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    t0 = rospy.get_time()
    outcome = sm.execute()
    rospy.loginfo("Gathering completed in %.2fs with outcome '%s'", rospy.get_time() - t0, outcome)

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

    rospy.signal_shutdown('All done.')
