import rospy
import smach
import smach_ros
import threading
import actionlib

import geometry_msgs.msg as geo_msgs
import rosgraph_msgs.msg as rosgraph_msgs

import mbf_msgs.msg as mbf_msgs
import thorp_msgs.msg as thorp_msgs
import cob_perception_msgs.msg as cob_msgs
import rail_manipulation_msgs.msg as rail_msgs
import rail_manipulation_msgs.srv as rail_srvs

from thorp_toolkit.geometry import pose2d2str, TF2


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
                rospy.loginfo("Detected " + detection.label)
                # I retort a bit MonitorState's logic; returning here False means 'invalid' msg, but it makes the state
                # to finish, that is what I want. MonitorState's execute will return 'invalid', but I just ignore it.
                # Returning True means keep receiving msgs, again what I want.
                return False
        return True

    def execute(self, ud):
        super(MonitorObjects, self).execute(ud)
        outcome = 'detected' if rospy.get_time() - self.detected_at <= self.persistence else 'not_detected'
        rospy.loginfo("Monitor ended with outcome " + outcome)
        return outcome


class MonitorTables(smach.State):
    """
    Look for tables until one is found or we run out of time. Returns:
    - 'detected' if a table is seen
    - 'not_detected' otherwise
    """

    def __init__(self):
        super(MonitorTables, self).__init__(output_keys=['detected_table', 'detected_table_pose'])
        self.table_sub = rospy.Subscriber("rail_segmentation/segmented_table", rail_msgs.SegmentedObject, self.table_cb)
        self.table_event = threading.Condition()
        self.segment_srv = rospy.ServiceProxy('rail_segmentation/segment_objects', rail_srvs.SegmentObjects)
        self.segment_srv.wait_for_service(30.0)
        self.detected_table = None

    def table_cb(self, msg):
        self.detected_table = msg
        self.table_event.acquire()
        self.table_event.notify()
        self.table_event.release()

    def execute(self, ud):
        self.detected_table = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.segment_srv()
            self.table_event.acquire()
            self.table_event.wait(0.1)
            self.table_event.release()
            if self.detected_table:
                print self.detected_table.point_cloud.header
                pose = geo_msgs.PoseStamped(self.detected_table.point_cloud.header,
                                            geo_msgs.Pose(self.detected_table.centroid, self.detected_table.orientation))
                ud['detected_table'] = self.detected_table
                ud['detected_table_pose'] = TF2().transform_pose(pose, pose.header.frame_id, 'map')
                rospy.loginfo("Detected table of size %.1fx%.1f at %s",
                              self.detected_table.width, self.detected_table.depth, pose2d2str(pose.pose))
                return 'detected'
            else:
                # nothing detected; TODO timeout   return 'not_detected'
                pass
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

