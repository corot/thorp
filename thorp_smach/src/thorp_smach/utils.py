import rospy
import smach_ros

from thorp_toolkit.common import wait_for_mbf, wait_for_sim_time
from thorp_toolkit.geometry import TF2


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
