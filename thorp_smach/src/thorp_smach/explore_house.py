#!/usr/bin/env python

import rospy

from thorp_smach.states.exploration import ExploreHouse

from thorp_smach.utils import run_sm


if __name__ == '__main__':
    rospy.init_node('explore_house_smach')

    run_sm(ExploreHouse(), rospy.get_param('~app_name'))
