import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import geometry_msgs.msg as geo_msgs

import cob_perception_msgs.msg as cob_msgs
from rail_manipulation_msgs.srv import SegmentObjects

from thorp_toolkit.geometry import point3d2str, pose2d2str, TF2, to_transform

from .. import config as cfg


class DetectObjects(smach_ros.SimpleActionState):
    """
    Detect objects state:
    Tries to segment a support surface and classify the tabletop objects found on it. Returns only the
    objects of types listed on 'object_types' input key (or all if it's not provided).
    As output, it returns the objects as a list of moveit_msgs/CollisionObject msgs and a single msg for
    the support surface.
    All detected objects and the support surface will be added to the planning scene as collision objects.
    If clear_scene is true, the planning scene will be previously emptied.
    """

    def __init__(self, clear_scene=False):
        super(DetectObjects, self).__init__('object_detection',
                                            thorp_msgs.DetectObjectsAction,
                                            goal_cb=self.goal_cb,
                                            result_cb=self.result_cb,
                                            input_keys=['object_types'],
                                            output_keys=['objects', 'surface'])
        self.clear_scene = clear_scene

    def goal_cb(self, ud, goal):
        goal.clear_scene = self.clear_scene

    def result_cb(self, ud, status, result):
        objects = result.objects
        if 'object_types' in ud and ud['object_types']:
            objects = [co for co in objects if co.id.split()[0] in ud['object_types']]
        ud['objects'] = objects
        ud['surface'] = result.surface


class ObjectsDetected(smach.State):
    """
    Check whether we have detected any object
    """

    def __init__(self):
        super(ObjectsDetected, self).__init__(outcomes=['true', 'false'],
                                              input_keys=['objects'],
                                              output_keys=['objects_count'])

    def execute(self, ud):
        objects_count = len(ud['objects'])
        ud['objects_count'] = objects_count
        return 'true' if objects_count > 0 else 'false'


class MonitorObjects(smach_ros.MonitorState):
    """
    Monitor tracked objects for a subset of objects. Returns:
    - 'succeeded' if anyone is seen within 'persistence' seconds
    - 'aborted' otherwise
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
        outcome = 'succeeded' if rospy.get_time() - self.detected_at <= self.persistence else 'aborted'
        rospy.loginfo("Monitor ended with outcome " + outcome)
        return outcome


class CheckTableSize(smach.State):
    """
    Check whether the given table dimensions are within the expected limits. Returns:
    - 'succeeded' if table dimensions are valid
    - 'aborted' otherwise
    """

    def __init__(self):
        super(CheckTableSize, self).__init__(outcomes=['succeeded', 'aborted'],
                                             input_keys=['table'])

    def execute(self, ud):
        center, width, length = ud['table'].center, ud['table'].width, ud['table'].depth
        if min(width, length) < cfg.TABLE_MIN_SIDE or max(width, length) > cfg.TABLE_MAX_SIDE:
            rospy.loginfo("Table at %s rejected due to invalid size: %.2f x %.2f", point3d2str(center), width, length)
            return 'aborted'
        return 'succeeded'


class MonitorTables(smach.State):
    """
    Look for tables until one is found, or we run out of time. Returns:
    - 'succeeded' if a table is seen before timeout (unlimited by default)
    - 'aborted' otherwise
    """

    def __init__(self, timeout=0.0):
        super(MonitorTables, self).__init__(outcomes=['succeeded', 'preempted', 'aborted'],
                                            input_keys=['table'],
                                            output_keys=['table', 'table_pose'])
        self.timeout = timeout
        self.segment_srv = rospy.ServiceProxy('rail_segmentation/segment_objects', SegmentObjects)
        self.segment_srv.wait_for_service(30.0)

    def execute(self, ud):
        start_time = rospy.get_time()
        rate = rospy.Rate(2.5)
        while not self.preempt_requested() and not rospy.is_shutdown():
            segmented_objs = self.segment_srv(only_surface=True).segmented_objects.objects
            if segmented_objs:
                table = segmented_objs[0]
                pose = geo_msgs.PoseStamped(table.point_cloud.header,
                                            geo_msgs.Pose(table.center, table.orientation))
                pose = TF2().transform_pose(pose, pose.header.frame_id, 'map')
                width, length = table.width, table.depth
                table_tf = to_transform(pose, 'table_frame')
                TF2().publish_transform(table_tf)
                if 'table' in ud:
                    table.name = ud['table'].name
                ud['table'] = table
                ud['table_pose'] = pose
                rospy.loginfo("Detected table of size %.1f x %.1f at %s", width, length, pose2d2str(pose))
                return 'succeeded'
            elif self.timeout and rospy.get_time() - start_time > self.timeout:
                rospy.logwarn("No table detected after %g seconds", rospy.get_time() - start_time)
                return 'aborted'
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        rospy.logwarn("Detect table has been preempted after %g seconds (timeout %g)",
                      rospy.get_time() - start_time, self.timeout)
        return 'preempted'


class ClearMarkers(smach_ros.ServiceState):
    def __init__(self):
        super(ClearMarkers, self).__init__('rail_segmentation/clear_markers', std_srvs.Empty)
