import rospy
import smach
import smach_ros
import threading

import thorp_msgs.msg as thorp_msgs
import geometry_msgs.msg as geo_msgs

from turtlebot_arm_block_manipulation.msg import BlockDetectionAction
import cob_perception_msgs.msg as cob_msgs
import rail_manipulation_msgs.msg as rail_msgs
from rail_manipulation_msgs.srv import SegmentObjects
from rail_mesh_icp.srv import TemplateMatch

from thorp_toolkit.semantic_map import SemanticMap
from thorp_toolkit.geometry import pose2d2str, TF2                   ,yaw,to_transform, quaternion_msg_from_yaw
from manipulation_states import FoldArm, ClearOctomap


def ObjectDetection():
    """  Object detection sub state machine; iterates over object_detection action state and recovery
         mechanism until an object is detected, it's preempted or there's an error (aborted outcome) """

    class ObjDetectedCondition(smach.State):
        """ Check for the object detection result to retry if no objects where detected """

        def __init__(self):
            smach.State.__init__(self, outcomes=['preempted', 'satisfied', 'fold_arm', 'retry'],
                                 input_keys=['od_attempt', 'object_names', 'table'],
                                 output_keys=['od_attempt', 'support_surf'])

        def execute(self, ud):
            ud['support_surf'] = ud['table'].id  # TODO ugly... cannot access subfields on ud?
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            if len(ud.object_names) > 0:
                ud.od_attempt = 0
                return 'satisfied'
            ud.od_attempt += 1
            if ud.od_attempt == 1:
                return 'fold_arm'
            return 'retry'

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                            input_keys=['od_attempt', 'output_frame'],
                            output_keys=['table', 'objects', 'object_names', 'support_surf'])

    with sm:
        smach.StateMachine.add('CLEAR_OCTOMAP', ClearOctomap(),
                               transitions={'succeeded': 'OBJECT_DETECTION',
                                            'preempted': 'preempted',
                                            'aborted': 'OBJECT_DETECTION'})

        smach.StateMachine.add('OBJECT_DETECTION',
                               smach_ros.SimpleActionState('object_detection',
                                                           thorp_msgs.DetectObjectsAction,
                                                           goal_slots=['output_frame'],
                                                           result_slots=['objects', 'object_names', 'support_surf']),
                               remapping={'support_surf': 'table'},
                               transitions={'succeeded': 'OBJ_DETECTED_COND',
                                            'preempted': 'preempted',
                                            'aborted': 'aborted'})

        smach.StateMachine.add('OBJ_DETECTED_COND',
                               ObjDetectedCondition(),
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


class BlockDetection(smach_ros.SimpleActionState):
    """
    Block detection state:
    use turtlebot_arm_block_manipulation demo's simple block detector until I have a proper object detection
    """

    def __init__(self):
        super(BlockDetection, self).__init__('block_detection',
                                             BlockDetectionAction,
                                             goal_cb=self.make_goal,
                                             result_cb=self.result_cb,
                                             output_keys=['blocks', 'objects', 'object_names', 'support_surf'])

    def make_goal(self, ud, goal):
        goal.frame = rospy.get_param('~arm_link', 'arm_base_link')
        goal.table_height = rospy.get_param('~table_height', -0.03)
        goal.block_size = rospy.get_param('~block_size', 0.025)

    def result_cb(self, ud, status, result):
        ud['blocks'] = result.blocks
        ud['objects'] = result.blocks
        # blocks only contains poses, so we need to invent values for 'object_names' and 'support_surf' fields
        # the real ObjectDetection state will provide more detailed information
        ud['object_names'] = ['block' + str(i) for i in range(1, len(result.blocks.poses) + 1)]
        ud['support_surf'] = 'table'


class ObjectsDetected(smach.State):
    """
    Check whether we have detected any object
    """

    def __init__(self):
        super(ObjectsDetected, self).__init__(outcomes=['true', 'false'],
                                              input_keys=['object_names'],
                                              output_keys=['objects_count'])

    def execute(self, ud):
        objects_count = len(ud['object_names'])
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


class TableWasVisited(smach.State):
    """
    Check whether a table has been visited before
    """

    def __init__(self):
        super(TableWasVisited, self).__init__(outcomes=['true', 'false'],
                                              input_keys=['table', 'table_pose'])

    def execute(self, ud):
        intersecting_objs = SemanticMap().objects_at(ud['table_pose'], (ud['table'].depth, ud['table'].width))
        outcome = 'true' if len(intersecting_objs) > 0 else 'false'
        print "TABLE VISITED???   ", outcome
        return outcome


class TableMarkVisited(smach.State):
    """
    Check whether we have detected any object
    """

    def __init__(self):
        super(TableMarkVisited, self).__init__(outcomes=['succeeded'],
                                               input_keys=['table', 'table_pose'])

    def execute(self, ud):
        SemanticMap().add_object(ud['table'], 'table', ud['table_pose'], (ud['table'].depth, ud['table'].width))
        return 'succeeded'


class MonitorTables(smach.State):
    """
    Look for tables until one is found or we run out of time. Returns:
    - 'succeeded' if a table is seen before timeout (unlimited by default)
    - 'aborted' otherwise
    """

    def __init__(self, timeout=0.0):
        super(ObjectDetectionRAIL, self).__init__(outcomes=['succeeded', 'preempted', 'aborted'],
                                                  output_keys=['table', 'table_pose'])
        self.timeout = timeout
        self.segment_srv = rospy.ServiceProxy('rail_segmentation/segment_objects', SegmentObjects)
        self.segment_srv.wait_for_service(30.0)

    def execute(self, ud):
        start_time = rospy.get_time()
        rate = rospy.Rate(1.0)
        while not self.preempt_requested() and not rospy.is_shutdown():
            segmented_objs = self.segment_srv().segmented_objects.objects
            if segmented_objs:
                table = segmented_objs[0]
                pose = geo_msgs.PoseStamped(table.point_cloud.header,
                                            geo_msgs.Pose(table.center, table.orientation))
                pose = TF2().transform_pose(pose, pose.header.frame_id, 'map')
                pose.pose.orientation = quaternion_msg_from_yaw(0)  #  assume tables aligned with x
                                                        # TODO restore once I fix RAIL to provide propper orientation
                width, length = table.width, table.depth
                table_tf = to_transform(pose, 'table_frame')
                TF2().publish_transform(table_tf)
                # print table.point_cloud.header
                # print 'centroid ', table.centroid
                # print 'center   ', table.center
                # print 'yaw      ', yaw(table.orientation)
                # kk=table.marker
                ud['table'] = table
                ud['table_pose'] = pose
                rospy.loginfo("Detected table of size %.1f x %.1f at %s", width, length, pose2d2str(pose))
                # Add the table contour as an obstacle to global costmap, so we can plan around it
                # We also add an shrinked version to the local costmap so no to impair approaching for picking
                # TODO: detected_table.name is empty; tweak RAIL to provide it or add sequential names here
                # SemanticLayer().add_obstacle(table.name, pose, [width, length, 0.0], 'global')
                # SemanticLayer().add_obstacle(table.name, pose, [width - 0.2, length - 0.2, 0.0], 'local')    TDDO  a ver q pasa
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



class ObjectDetectionRAIL(smach.State):
    """
    Look for tables until one is found or we run out of time. Returns:
    - 'succeeded' if a table is seen before timeout (unlimited by default)
    - 'aborted' otherwise
    """

    def __init__(self, timeout=0.0):
        super(ObjectDetectionRAIL, self).__init__(outcomes=['succeeded', 'preempted', 'aborted'],
                                                  output_keys=['table', 'table_pose', 'objects'])
        self.templates = ['clover'] #, 'clover', 'triangle', 'tower']
        self.templates = ['tower',
                          'cube',
                          'square',
                          'rectangle',
                          'triangle',
                          'pentagon',
                          'circle',
                          'star',
                          'diamond',
                          'cross',
                          'clover']
        self.timeout = timeout
        self.segment_srv = rospy.ServiceProxy('rail_segmentation/segment_objects', SegmentObjects)
        self.segment_srv.wait_for_service(30.0)
        self.match_srvs = {}
        for template in self.templates:
            self.match_srvs[template] = rospy.ServiceProxy('/template_matcher_%s/match_template' % template,
                                                           TemplateMatch)
            self.match_srvs[template].wait_for_service(30.0)

    def execute(self, ud):
        start_time = rospy.get_time()
        segmented_objs = self.segment_srv().segmented_objects.objects
        if len(segmented_objs) > 1:
            table = segmented_objs[0]
            pose = geo_msgs.PoseStamped(table.point_cloud.header,
                                        geo_msgs.Pose(table.center, table.orientation))
            pose.header.stamp.set(0, 0)
            pose = TF2().transform_pose(pose, pose.header.frame_id, 'map')
            pose.pose.orientation = quaternion_msg_from_yaw(0)  #  assume tables aligned with x
                                                    # TODO restore once I fix RAIL to provide propper orientation
            width, length = table.width, table.depth
            table_tf = to_transform(pose, 'table_frame')
            TF2().publish_transform(table_tf)
            # print table.point_cloud.header
            # print 'centroid ', table.centroid
            # print 'center   ', table.center
            # print 'yaw      ', yaw(table.orientation)
            # kk=table.marker
            ud['table'] = table
            ud['table_pose'] = pose
            rospy.loginfo("Detected table of size %.1f x %.1f at %s", width, length, pose2d2str(pose))
            # Add the table contour as an obstacle to global costmap, so we can plan around it
            # We also add an shrinked version to the local costmap so no to impair approaching for picking
            # TODO: detected_table.name is empty; tweak RAIL to provide it or add sequential names here
            # SemanticLayer().add_obstacle(table.name, pose, [width, length, 0.0], 'global')
            # SemanticLayer().add_obstacle(table.name, pose, [width - 0.2, length - 0.2, 0.0], 'local')    TDDO  a ver q pasa
            for segmented_obj in segmented_objs[1:]:
                initial_estimate = geo_msgs.Transform(segmented_obj.center, segmented_obj.orientation)
                initial_estimate.translation.z -= segmented_obj.height / 2.0  # our templates' base is at z = 0
                #####initial_estimate.rotation = quaternion_msg_from_yaw(0.0)
                print (yaw(initial_estimate.rotation)*180)/3.1416
                min_error = float('inf')
                best_fit = None
                for template in self.templates:
                    # print segmented_obj.center.z
                    # print segmented_obj.centroid.z
                    #segmented_obj.center.z = table.center.z + table.height
                    # print segmented_obj.center.z
                    # segmented_obj.center.z = table.centroid.z + table.height
                    # print segmented_obj.center.z
                    resp = self.match_srvs[template](initial_estimate, segmented_obj.point_cloud)
                    print template, resp.match_error, rospy.get_time() - start_time
                    if resp.match_error < min_error:
                        min_error = resp.match_error
                        best_fit = template
                print  best_fit, min_error
        return 'succeeded'
        # elif self.timeout and rospy.get_time() - start_time > self.timeout:
        #     rospy.logwarn("No table detected after %g seconds", rospy.get_time() - start_time)
        #     return 'aborted'
        # try:
        #     rate.sleep()
        # except rospy.ROSInterruptException:
        #     break
        # rospy.logwarn("Detect table has been preempted after %g seconds (timeout %g)",
        #               rospy.get_time() - start_time, self.timeout)
        # return 'preempted'
