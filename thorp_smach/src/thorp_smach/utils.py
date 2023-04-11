import rospy
import smach
import smach_ros

from thorp_toolkit.common import wait_for_mbf, wait_for_sim_time
from thorp_toolkit.geometry import TF2


def run_sm(sm, name, parent_ud=smach.UserData()):
    """
    Run the given state machine
    """
    TF2()  # start listener asap to avoid delays when running

    rospy.sleep(rospy.get_param('~start_delay', 0.0))

    wait_for_sim_time()

    # Create and start the introspection server, if required (for FSM visualization or for controlling the camera)
    if rospy.get_param('~run_introspection_server', False):
        sis = smach_ros.IntrospectionServer(name, sm, '/SM_ROOT')
        sis.start()

    # MBF is the last component to start, so wait for it before running the sm
    wait_for_mbf()

    # Execute the state machine
    t0 = rospy.get_time()
    outcome = sm.execute(parent_ud)
    rospy.loginfo("%s completed in %.2fs with outcome '%s'", name, rospy.get_time() - t0, outcome)

    # Wait for ctrl-c to stop the application
    rospy.spin()
    if 'sis' in locals():
        sis.stop()

    rospy.signal_shutdown(name + ' ended')
