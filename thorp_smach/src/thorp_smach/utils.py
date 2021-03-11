import rospy
import smach_ros
import actionlib

import rosgraph_msgs.msg as rosgraph_msgs

import mbf_msgs.msg as mbf_msgs

from thorp_toolkit.geometry import TF2


def wait_for_sim_time():
    """
    In sim, wait for clock to start (I start gazebo paused, so smach action clients start waiting at time 0,
    but first clock marks ~90s, after spawner unpauses physics)
    """
    if rospy.get_param('/use_sim_time', False):
        if not rospy.wait_for_message('/clock', rosgraph_msgs.Clock, rospy.Duration(60)):
            rospy.logfatal("No clock msgs after 60 seconds, being use_sim_time true")
            return False
    return True


def wait_for_mbf():
    """
    Wait for Move Base Flex's move_base action (the last to be started) getting available
    """
    mb_ac = actionlib.SimpleActionClient("/move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    available = mb_ac.wait_for_server(rospy.Duration(30))
    if not available:
        rospy.logwarn("Move Base Flex not available after 30 seconds")
    return available


def run_sm(sm, name):
    """
    Run the given state machine
    """
    TF2()  # start listener asap to avoid delays when running

    wait_for_sim_time()

    # Create and start the introspection server
    if rospy.get_param('~viz_smach', True):
        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

    # MBF is the last component to start, so wait for it before running the sm
    wait_for_mbf()

    # Execute the state machine
    t0 = rospy.get_time()
    outcome = sm.execute()
    rospy.loginfo("%s completed in %.2fs with outcome '%s'", name, rospy.get_time() - t0, outcome)

    # Wait for ctrl-c to stop the application
    rospy.spin()
    if 'sis' in locals():
        sis.stop()

    rospy.signal_shutdown('All done.')
