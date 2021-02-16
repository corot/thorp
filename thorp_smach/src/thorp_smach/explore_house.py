#!/usr/bin/env python

import rospy

from toolkit.common_states import run_sm
from toolkit.exploration_states import ExploreHouse


if __name__ == '__main__':
    rospy.init_node('explore_house_smach')

    run_sm(ExploreHouse(), rospy.get_param('~app_name'))
