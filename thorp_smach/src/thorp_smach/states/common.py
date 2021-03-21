import rospy
import smach

import thorp_msgs.msg as thorp_msgs

from thorp_toolkit.reconfigure import Reconfigure


class SetNamedConfig(smach.State):
    """
    Set a given named configuration. Returns
    - 'true' if succeeded
    - 'false' otherwise
    """
    def __init__(self, config_name):
        super(SetNamedConfig, self).__init__(outcomes=['succeeded', 'aborted'])
        self.config_name = config_name

    def execute(self, ud):
        return 'succeeded' if Reconfigure().use_named_config(self.config_name) else 'aborted'


class DismissNamedConfig(smach.State):
    """
    Dismiss a given named configuration. Returns
    - 'true' if succeeded
    - 'false' otherwise
    """
    def __init__(self, config_name):
        super(DismissNamedConfig, self).__init__(outcomes=['succeeded', 'aborted'])
        self.config_name = config_name

    def execute(self, ud):
        return 'succeeded' if Reconfigure().dismiss_named_config(self.config_name) else 'aborted'


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
