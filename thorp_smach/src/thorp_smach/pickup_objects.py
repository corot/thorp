#!/usr/bin/env python

import rospy

from thorp_smach.states.pickup_objs import PickupReachableObjs

from thorp_smach.utils import run_sm


if __name__ == '__main__':
    rospy.init_node('pickup_objects_smach')

    run_sm(PickupReachableObjs(), rospy.get_param('~app_name'))
