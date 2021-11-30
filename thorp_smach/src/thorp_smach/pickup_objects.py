#!/usr/bin/env python

import rospy

from thorp_smach.states.gathering import PickReachableObjs

from thorp_smach.utils import run_sm


if __name__ == '__main__':
    rospy.init_node('pickup_objects_smach')

    run_sm(PickReachableObjs(), rospy.get_param('~app_name'))
