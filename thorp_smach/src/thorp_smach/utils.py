import rospy
import smach
import smach_ros

from thorp_toolkit.common import wait_for_mbf, wait_for_sim_time
from thorp_toolkit.geometry import TF2


def run_sm(sm, name, asw=None, parent_ud=smach.UserData()):
    """
    Run the given state machine
    :param sm: Target state machine
    :param name: State machine name
    :param asw: Action server wrapper; used to interact with the target sm instead of just calling execute
    :param parent_ud: Parent state machine userdata
    """
    rospy.sleep(rospy.get_param('~start_delay', 0.0))

    wait_for_sim_time()

    # Create and start the introspection server, if required (for FSM visualization or for controlling the camera)
    if rospy.get_param('~run_introspection_server', False):
        sis = smach_ros.IntrospectionServer(name, sm, '/SM_ROOT')
        sis.start()

    # MBF is the last component to start, so wait for it before running the sm
    wait_for_mbf()

    if asw is not None:
        # If provided, run the action server wrapper in a background thread
        asw.run_server()
    else:
        # Otherwise execute the state machine
        t0 = rospy.get_time()
        outcome = sm.execute(parent_ud)
        rospy.loginfo("%s completed in %.2fs with outcome '%s'", name, rospy.get_time() - t0, outcome)

    # Wait for ctrl-c to stop the application
    rospy.spin()
    if 'sis' in locals():
        sis.stop()

    rospy.signal_shutdown(name + ' ended')
