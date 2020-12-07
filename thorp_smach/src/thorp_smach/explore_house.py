#!/usr/bin/env python

import rospy
import smach_ros

from toolkit.exploration_states import ExploreHouse


if __name__ == '__main__':
    rospy.init_node('explore_house_smach')

    sm = ExploreHouse()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    t0 = rospy.get_time()
    outcome = sm.execute()
    rospy.loginfo("Exploration completed in %.2fs with outcome '%s'", rospy.get_time() - t0, outcome)

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

    rospy.signal_shutdown('All done.')
