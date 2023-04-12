#!/usr/bin/env python

import rospy
import smach

from thorp_smach.states.common import ExecuteFn
from thorp_smach.states.exploration import ExploreHouse

from thorp_smach.utils import run_sm


if __name__ == '__main__':
    rospy.init_node('explore_house_smach')

    explore_house_sm = ExploreHouse()
    explore_house_once = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                        connector_outcome='succeeded')
    with explore_house_once:
        smach.Sequence.add('EXPLORE_HOUSE_ONCE', explore_house_sm)
        smach.Sequence.add('RESET_COMPLETED_ROOMS', ExecuteFn(explore_house_sm.reset_completed_rooms))

    # iterate a number of house exploration laps
    explore_house_it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                                      input_keys=[],
                                      output_keys=[],
                                      it=lambda: range(rospy.get_param('~laps', 3)),
                                      it_label='lap',
                                      exhausted_outcome='succeeded')
    with explore_house_it:
        smach.Iterator.set_contained_state('EXPLORE_HOUSE', explore_house_once, loop_outcomes=['succeeded'])

    run_sm(explore_house_it, rospy.get_param('~app_name'))
