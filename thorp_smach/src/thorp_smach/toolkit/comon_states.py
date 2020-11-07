import rospy
import smach
import actionlib

import rosgraph_msgs.msg as rosgraph_msgs

import mbf_msgs.msg as mbf_msgs
import thorp_msgs.msg as thorp_msgs


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


class UDHasKey(smach.State):
    """
    Check if our userdata contains a given key. Returns
    - 'true' if present
    - 'false' otherwise
    """

    def __init__(self, key):
        super(UDHasKey, self).__init__(outcomes=['true', 'false'],
                                       input_keys=[key])
        self.key = key

    def execute(self, ud):
        try:
            ud[self.key]
            return 'true'
        except KeyError:
            return 'false'


class ExecuteUserCommand(smach.State):
    """ Different starts of the SM depending on the command provided when calling the actionlib wrapper """

    def __init__(self, valid_commands):
        self.valid_commands = valid_commands
        smach.State.__init__(self, outcomes=['invalid_command'] + valid_commands,
                             input_keys=['user_command'],
                             output_keys=['ucmd_progress', 'ucmd_outcome'])

    def execute(self, ud):
        if ud['user_command'].command in self.valid_commands:
            rospy.loginfo("Executing User Command '%s'", ud['user_command'].command)
            ud['ucmd_progress'] = thorp_msgs.UserCommandFeedback('executing_command')
            return ud['user_command'].command
        else:
            rospy.logwarn("Invalid User Command: '%s'", ud['user_command'].command)
            ud['ucmd_outcome'] = thorp_msgs.UserCommandResult('invalid_command')
            return 'invalid_command'
