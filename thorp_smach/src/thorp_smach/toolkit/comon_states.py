import rospy
import smach
import smach_ros

import thorp_msgs.msg as thorp_msgs
import cob_perception_msgs.msg as cob_msgs


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


class MonitorObjects(smach_ros.MonitorState):
    """
    Monitor tracked objects for a subset of objects. Returns:
    - 'detected' if anyone is seen within 'persistence' seconds
    - 'not_detected' otherwise
    """

    def __init__(self, objects):
        super(MonitorObjects, self).__init__("tracked_objects",
                                             cob_msgs.DetectionArray,
                                             self.monitor_cb,
                                             output_keys=['tracked_object', 'tracked_object_pose'])
        self.objects = objects
        self.persistence = 1.0
        self.detected_at = - self.persistence

    def monitor_cb(self, ud, msg):
        for detection in msg.detections:
            if detection.label in self.objects:
                self.detected_at = rospy.get_time()
                ud['tracked_object'] = detection
                ud['tracked_object_pose'] = detection.pose
                return False
        return True

    def execute(self, ud):
        super(MonitorObjects, self).execute(ud)
        print 'detected' if rospy.get_time() - self.detected_at <= self.persistence else 'not_detected'
        return 'detected' if rospy.get_time() - self.detected_at <= self.persistence else 'not_detected'
