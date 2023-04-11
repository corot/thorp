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


class Sleep(smach.State):
    """
    Sleeps the given time. Returns
    - 'succeeded' if we slept the specified time
    - 'aborted' if no time is provided
    """

    def __init__(self, time=None):
        super(Sleep, self).__init__(outcomes=['succeeded', 'aborted', 'preempted'],
                                    input_keys=['time'])
        self.time = time

    def execute(self, ud):
        if self.time is not None:
            time = self.time
        elif 'time' in ud:
            time = ud['time']
        else:
            rospy.logerr("Sleep time not provided")
            return 'aborted'
        rospy.loginfo("Sleeping for %g seconds...", time)
        t0 = rospy.get_time()
        while rospy.get_time() - t0 < time:
            if self.preempt_requested():
                return 'preempted'
            rospy.sleep(0.001)
        return 'succeeded'


class Succeed(smach.State):
    """
    Simply succeed
    """

    def __init__(self):
        super(Succeed, self).__init__(outcomes=['succeeded'])

    def execute(self, _):
        return 'succeeded'


class Fail(smach.State):
    """
    Simply fail
    """

    def __init__(self):
        super(Fail, self).__init__(outcomes=['aborted'])

    def execute(self, _):
        return 'aborted'


class ExecuteFn(smach.State):
    """
    Execute a callable object. Returns
    - 'succeeded' if the passed object is callable
    - 'aborted' otherwise
    """

    def __init__(self, fn):
        super(ExecuteFn, self).__init__(outcomes=['succeeded', 'aborted'])
        self.fn = fn

    def execute(self, ud):
        if callable(self.fn):
            self.fn()
            return 'succeeded'
        rospy.logerr("Trying to run non callable object '%s'", str(self.fn))
        return 'aborted'
