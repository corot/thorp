import rospy
import smach
import smach_ros
import threading

import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import geometry_msgs.msg as geo_msgs

import cob_perception_msgs.msg as cob_msgs
import rail_manipulation_msgs.msg as rail_msgs
import rail_manipulation_msgs.srv as rail_srvs

from thorp_toolkit.geometry import pose2d2str, TF2                   ,yaw,to_transform, quaternion_msg_from_yaw
from thorp_toolkit.semantic_map import SemanticMap
from manipulation_states import FoldArm


def ObjectDetection():
    """  Object detection sub state machine; iterates over object_detection action state and recovery
         mechanism until an object is detected, it's preempted or there's an error (aborted outcome) """

    class ObjDetectedCondition(smach.State):
        """ Check for the object detection result to retry if no objects where detected """

        def __init__(self):
            smach.State.__init__(self, outcomes=['preempted', 'satisfied', 'fold_arm', 'retry'],
                                 input_keys=['od_attempt', 'object_names'],
                                 output_keys=['od_attempt'])

        def execute(self, userdata):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            if len(userdata.object_names) > 0:
                userdata.od_attempt = 0
                return 'satisfied'
            userdata.od_attempt += 1
            if userdata.od_attempt == 1:
                return 'fold_arm'
            return 'retry'

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                            input_keys=['od_attempt', 'output_frame'],
                            output_keys=['objects', 'object_names', 'support_surf'])

    with sm:
        smach.StateMachine.add('CLEAR_OCTOMAP',
                               smach_ros.ServiceState('clear_octomap',
                                                      std_srvs.Empty),
                               transitions={'succeeded': 'OBJECT_DETECTION',
                                            'preempted': 'preempted',
                                            'aborted': 'OBJECT_DETECTION'})

        smach.StateMachine.add('OBJECT_DETECTION',
                               smach_ros.SimpleActionState('object_detection',
                                                           thorp_msgs.DetectObjectsAction,
                                                           goal_slots=['output_frame'],
                                                           result_slots=['objects', 'object_names', 'support_surf']),
                               remapping={'output_frame': 'output_frame',
                                          'object_names': 'object_names',
                                          'support_surf': 'support_surf'},
                               transitions={'succeeded': 'OBJ_DETECTED_COND',
                                            'preempted': 'preempted',
                                            'aborted': 'aborted'})

        smach.StateMachine.add('OBJ_DETECTED_COND',
                               ObjDetectedCondition(),
                               remapping={'object_names': 'object_names'},
                               transitions={'satisfied': 'succeeded',
                                            'preempted': 'preempted',
                                            'fold_arm': 'FOLD_ARM',
                                            'retry': 'CLEAR_OCTOMAP'})

        smach.StateMachine.add('FOLD_ARM',
                               FoldArm(),
                               transitions={'succeeded': 'CLEAR_OCTOMAP',
                                            'preempted': 'preempted',
                                            'aborted': 'CLEAR_OCTOMAP'})

    return sm


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


class MonitorTables(smach.State):
    """
    Look for tables until one is found or we run out of time. Returns:
    - 'detected' if a table is seen
    - 'aborted' otherwise
    """

    def __init__(self):
        super(MonitorTables, self).__init__(outcomes=['succeeded', 'aborted'],
                                            output_keys=['detected_table', 'detected_table_pose'])
        self.detected_table = None
        self.table_event = threading.Condition()
        self.table_sub = rospy.Subscriber("rail_segmentation/segmented_table", rail_msgs.SegmentedObject, self.table_cb)
        self.segment_srv = rospy.ServiceProxy('rail_segmentation/segment_objects', rail_srvs.SegmentObjects)
        self.segment_srv.wait_for_service(30.0)

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
                pose = geo_msgs.PoseStamped(self.detected_table.point_cloud.header,
                                            geo_msgs.Pose(self.detected_table.center, self.detected_table.orientation))
                pose = TF2().transform_pose(pose, pose.header.frame_id, 'map')
                pose.pose.orientation = quaternion_msg_from_yaw(0)  #  assume tables aligned with x
                                                        # TODO restore once I fix RAIL to provide propper orientation
                width, length = self.detected_table.width, self.detected_table.depth
                table_tf = to_transform(pose, 'table_frame')
                TF2().publish_transform(table_tf)
                ud['detected_table'] = self.detected_table
                ud['detected_table_pose'] = pose
                rospy.loginfo("Detected table of size %.1f x %.1f at %s", width, length, pose2d2str(pose))
                # Add the table contour as an obstacle to global costmap, so we can plan around it
                # We don't add to the local costmap because that would impair approaching for picking
                # TODO: detected_table.name is empty; tweak RAIL to provide it or add sequential names here
                SemanticMap().add_obstacle(self.detected_table.name, pose, [width, length, 0.0], 'global')
                return 'succeeded'
            else:
                # nothing detected; TODO timeout   return 'not_detected'
                pass
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
