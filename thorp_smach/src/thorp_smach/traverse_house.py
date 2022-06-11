#!/usr/bin/env python

from thorp_smach.states.exploration import *
from thorp_smach.utils import run_sm


def traverse_house_sm():
    """
    Traverse house SM:
     - segment map into rooms and plan visit sequence
     - Go to each room following the planned sequence
    """
    # segment house into rooms and plan visit sequence
    plan_room_seq_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                      connector_outcome='succeeded',
                                      output_keys=['map_image', 'map_resolution', 'map_origin', 'robot_radius',
                                                   'segmented_map', 'room_sequence', 'room_information_in_meter'])
    with plan_room_seq_sm:
        smach.Sequence.add('SEGMENT_ROOMS', SegmentRooms())
        smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
        smach.Sequence.add('PLAN_ROOM_SEQUENCE', PlanRoomSequence(),
                           remapping={'input_map': 'map_image'})

    # explore a single room
    explore_1_room_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                       connector_outcome='succeeded',
                                       input_keys=['map_image', 'map_resolution', 'map_origin', 'robot_radius',
                                                   'segmented_map', 'room_number', 'room_information_in_meter'])
    with explore_1_room_sm:
        smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
        smach.Sequence.add('PLAN_ROOM_EXPL', PlanRoomExploration())
        smach.Sequence.add('GOTO_START_POSE', GoToPose(),#dist_tolerance=cfg.LOOSE_DIST_TOLERANCE,   # just close enough
                                                       #angle_tolerance=cfg.INF_ANGLE_TOLERANCE),  # ignore orientation
                           remapping={'target_pose': 'start_pose'})
                                      # 'dist_tolerance': 'loose_dist_tolerance',   # just close enough
                                      # 'angle_tolerance': 'inf_angle_tolerance'})  # ignore orientation
        # smach.Sequence.add('TRAVERSE_POSES', TraversePoses(),
        #                    remapping={'poses': 'coverage_path_pose_stamped',
        #                               'dist_tolerance': 'loose_dist_tolerance',   # just close enough
        #                               'angle_tolerance': 'inf_angle_tolerance'})  # ignore orientation
        # smach.Sequence.add('TRAVERSE_POSES', ExeSparsePath(),
        #                    remapping={'path': 'coverage_path_pose_stamped'})

    # iterate over all rooms and explore following the planned sequence
    explore_house_it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                                      input_keys=['map_image', 'map_resolution', 'map_origin', 'robot_radius',
                                                  'segmented_map', 'room_sequence', 'room_information_in_meter'],
                                      output_keys=[],
                                      it=lambda: sm.userdata.room_sequence,  # must be a lambda because we destroy the list  TODO  ehhh???
                                      it_label='room_number',
                                      exhausted_outcome='succeeded')
    with explore_house_it:
        smach.Iterator.set_contained_state('EXPLORE_1_ROOM', explore_1_room_sm, loop_outcomes=['succeeded', 'aborted'])

    # Full SM: plan rooms visit sequence and explore each room in turn
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with sm:
        smach.StateMachine.add('PLAN_ROOM_SEQ', plan_room_seq_sm,
                               transitions={'succeeded': 'EXPLORE_HOUSE',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('EXPLORE_HOUSE', explore_house_it,
                               transitions={'succeeded': 'EXPLORE_HOUSE1',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'}),
        smach.StateMachine.add('EXPLORE_HOUSE1', explore_house_it,
                               transitions={'succeeded': 'EXPLORE_HOUSE11',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'}),
        smach.StateMachine.add('EXPLORE_HOUSE11', explore_house_it,
                               transitions={'succeeded': 'EXPLORE_HOUSE111',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'}),
        smach.StateMachine.add('EXPLORE_HOUSE111', explore_house_it,
                               transitions={'succeeded': 'EXPLORE_HOUSE1111',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'}),
        smach.StateMachine.add('EXPLORE_HOUSE1111', explore_house_it,
                               transitions={'succeeded': 'succeeded',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
    return sm


if __name__ == '__main__':
    rospy.init_node('traverse_house_smach')

    run_sm(traverse_house_sm(), rospy.get_param('~app_name'))
