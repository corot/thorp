#!/usr/bin/env python

import rospy
import smach
import smach_ros

from toolkit.explore_states import *


def main():
    """
    Explore house SM:
     - segment map into rooms and plan visit sequence
     - iterate over all rooms and explore following the planned sequence
    """
    rospy.init_node('smach_explore_house')

    TF2()  # start listener asap

    # segment house into rooms and plan visit sequence
    plan_room_seq_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                      connector_outcome='succeeded',
                                      output_keys=['map_image', 'map_resolution', 'map_origin', 'robot_radius',
                                                   'segmented_map', 'room_sequence'])
    with plan_room_seq_sm:
        smach.Sequence.add('SEGMENT_ROOMS', SegmentRooms())
        smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
        smach.Sequence.add('PLAN_ROOM_SEQUENCE', PlanRoomSequence(),
                           remapping={'input_map': 'map_image'})

    # explore a single room
    explore_1_room_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                       connector_outcome='succeeded',
                                       input_keys=['map_image', 'map_resolution', 'map_origin', 'robot_radius',
                                                   'segmented_map', 'room_number'])
    with explore_1_room_sm:
        smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
        smach.Sequence.add('PLAN_ROOM_EXPL', PlanRoomExploration())
        smach.Sequence.add('TRAVERSE_POSES', TraversePoses(),
                           remapping={'poses': 'coverage_path_pose_stamped'})

    # iterate over all rooms and explore following the planned sequence
    explore_house_it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                                      input_keys=['map_image', 'map_resolution', 'map_origin', 'robot_radius',
                                                  'segmented_map', 'room_sequence'],
                                      output_keys=[],
                                      it=lambda: sm.userdata.room_sequence,  # must be a lambda because we destroy the list
                                      it_label='room_number',
                                      exhausted_outcome='succeeded')
    with explore_house_it:
        cont_sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                     input_keys=['map_image', 'map_resolution', 'map_origin', 'robot_radius',
                                                 'segmented_map', 'room_number'],
                                     output_keys=[])
        with cont_sm:
            smach.StateMachine.add('EXPLORE_1_ROOM', explore_1_room_sm,
                                   transitions={'succeeded': 'continue',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})

        smach.Iterator.set_contained_state('', cont_sm, loop_outcomes=['continue'])

    # Full SM: plan rooms visit sequence and explore each room in turn
    sm = smach.StateMachine(outcomes=['succeeded',
                                      'aborted',
                                      'preempted'],
                            output_keys=[])
    with sm:
        smach.StateMachine.add('PLAN_ROOM_SEQ', plan_room_seq_sm,
                               transitions={'succeeded': 'EXPLORE_HOUSE',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('EXPLORE_HOUSE', explore_house_it,
                               transitions={'succeeded': 'succeeded',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})

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
