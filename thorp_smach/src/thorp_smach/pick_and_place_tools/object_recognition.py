#!/usr/bin/env python
import tf
import math
from thorp_smach.state_machines.state_machines_imports import *
from thorp_smach.pick_and_place_tools.msg_imports import *

#===============================================================================================================
# Detect object callbacks
#===============================================================================================================
def objectRecognitionGoalCb(userdata, goal):
    goal = object_recognition_msgs.ObjectRecognitionGoal()
    return goal

@smach.cb_interface(input_keys=['min_confidence',
                                'recognised_objects',
                                 'object_names',
                                'error_code',
                                'error_message',
                                'tf_listener'],
                    output_keys=['recognised_objects',
                                 'object_names',
                                 'error_code',
                                 'error_message',
                                 'tf_listener'],
                    outcomes=['succeeded',
                              'no_objects_found',
                              'preempted',
                              'aborted'])
def objectRecognitionResultCb(userdata, status, result):
    if status == actionlib_msgs.GoalStatus.SUCCEEDED:
        reduced_model_set_dict = {'18665':'000.580.67',
                                  '18685':'501.245.12',
                                  '18691':'800.572.57',
                                  '18693':'801.327.80',
                                  '18699':'901.334.73',
                                  '18744':'coke_classic',
                                  '18746':'mach3_gel',
                                  '18765':'coffee-mate',
                                  '18766':'suave-kids-3in1',
                                  '18783':'izze_can',
                                  '18791':'v8_bottle',
                                  '18798':'tennis_ball_can',
                                  '18799':'clearasil_jar',
                                  '18800':'contact_lens_cleaner',
                                  '18802':'hydrogen_peroxide_bottle',
                                  '18807':'campbell_soup_can',
                                  '18808':'campbell_soup_handheld'}
        rospy.logdebug("Object recognition was successful. Processing the result ...")
        userdata.recognised_objects = object_recognition_msgs.RecognizedObjectArray()
        userdata.object_names = list()
        unreliable_recognition = int()
        for object in result.recognized_objects.objects:
            if object.confidence >= userdata.min_confidence:
                if object.type.key in reduced_model_set_dict:
                    userdata.object_names.append(reduced_model_set_dict[object.type.key])
                    rospy.loginfo("Added recognised object '" + reduced_model_set_dict[object.type.key] + "' ("
                                   + object.type.key + ") with confidence "
                                   + str(object.confidence))
                    ''' convert poses to robot_root since 3d sensor pose keeps changing '''
                    try:
                        new_pose = geometry_msgs.PoseStamped()
                        new_pose.header = object.pose.header
                        new_pose.pose = object.pose.pose.pose
                        new_pose.header.stamp = userdata.tf_listener.getLatestCommonTime(
                                                               "base_footprint", object.pose.header.frame_id)
                        new_pose = userdata.tf_listener.transformPose("base_footprint", new_pose)
                        object.pose.header = new_pose.header
                        object.pose.pose.pose = new_pose.pose
                    except tf.Exception, e:
                        rospy.logerr('Couldn`t transform requested pose!')
                        rospy.logerr('%s', e)
                    userdata.recognised_objects.objects.append(object)
                else:
                    userdata.error_message = "Object ID '" + str(object.type.key) + "' not in dictionary!"
                    rospy.logerr(userdata.error_message)
                    return 'aborted'
            else:
                unreliable_recognition += 1
                rospy.loginfo("Recognised object '" + reduced_model_set_dict[object.type.key] + "' ("
                                   + object.type.key + "), but confidence is too low ("
                                   + str(reduced_model_set_dict[object.type.key]) + ").")
        if len(result.recognized_objects.objects) == 0:
            userdata.error_message = "No objects found"
            rospy.loginfo(userdata.error_message)
            return 'no_objects_found'
        elif unreliable_recognition == len(result.recognized_objects.objects):
            userdata.error_message = "Objects found, but recognition was not reliable."
            rospy.loginfo(userdata.error_message)
            return 'no_objects_found'
        else:
            userdata.error_message = "Object recognition succeeded."
            rospy.loginfo(userdata.error_message)
            return 'succeeded'
    elif status == actionlib_msgs.GoalStatus.PREEMPTED:
        userdata.error_message = "Object recognition was preempted."
        rospy.logwarn(userdata.error_message)
        return 'preempted'
    else:
        userdata.error_message = "Object recognition failed!"
        rospy.logerr(userdata.error_message)
        return 'aborted'

class GetObjectInformation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done'],
                             input_keys=['recognised_objects',
                                         'error_code',
                                         'error_message'],
                             output_keys=['objects_info',
                                          'error_code',
                                          'error_message'])
    def execute(self, userdata):
        rospy.loginfo("Waiting for 'get_object_info' service ... ")
        rospy.wait_for_service("get_object_info")
        rospy.loginfo("'get_object_info' service available.")
        objects_info = list()
        srv_client = rospy.ServiceProxy("get_object_info",
                                         object_recognition_srvs.GetObjectInformation())
        for object in userdata.recognised_objects.objects:
            try:
                info_request = object_recognition_srvs.GetObjectInformationRequest()
                info_request.type = object.type
                rospy.loginfo("Getting information for: " + str(info_request))
                objects_info.append(srv_client(info_request))
            except rospy.ServiceException, e:
                rospy.logerr("Service did not process request: " + str(e))
        userdata.objects_info = objects_info
        return 'done'


class GetTable(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['table_added',
                                       'no_table_added'],
                             input_keys=['tabletop_centre_pose',
                                         'tabletop_front_place_pose',
                                         'tf_listener'],
                             output_keys=['tabletop_centre_pose',
                                          'tabletop_front_place_pose'
                                          'tf_listener'])
        self._pub_collision_object = None
        self._sub_table_markers = None
        self._table_markers = None
        self._new_table_markers_received = False

        self._table_width_enlargement = 0.10
        self._table_length_enlargement = 0.10
        self._table_height_enlargement = 0.05
        self._front_place_pose_factor = 0.75

    def _marker_cb(self, data):
        if not self._new_table_markers_received:
            rospy.loginfo("Received new table markers.")
            self._table_markers = data.markers
            self._new_table_markers_received = True

    def execute(self, userdata):
        self._pub_collision_object = rospy.Publisher("collision_object",
                                                     moveit_msgs.CollisionObject,
                                                     latch=True)
        self._sub_table_markers = rospy.Subscriber("marker_tables",
                                                   visualization_msgs.MarkerArray,
                                                   self._marker_cb)
        tf_transformer = tf.Transformer()
        tf_transform_listener = userdata.tf_listener
        map_root = "base_footprint"
        rate = rospy.Rate(1)
        table_added = False
        markers_processed = False

        while not rospy.is_shutdown() and not markers_processed:
            if self._new_table_markers_received:
                for marker in self._table_markers:
                    try:
                        ''' transform marker frame into map_root '''
                        tf_time = tf_transform_listener.getLatestCommonTime(marker.header.frame_id, map_root)
                        translation, rotation = tf_transform_listener.lookupTransform(map_root,
                                                                                       marker.header.frame_id,
                                                                                       tf_time)
                        tf_odom_marker_frame = geometry_msgs.TransformStamped()
                        tf_odom_marker_frame.header.stamp = tf_time
                        tf_odom_marker_frame.header.frame_id = map_root
                        tf_odom_marker_frame.child_frame_id = marker.header.frame_id
                        tf_odom_marker_frame.transform.translation.x = translation[0]
                        tf_odom_marker_frame.transform.translation.y = translation[1]
                        tf_odom_marker_frame.transform.translation.z = translation[2]
                        tf_odom_marker_frame.transform.rotation.x = rotation[0]
                        tf_odom_marker_frame.transform.rotation.y = rotation[1]
                        tf_odom_marker_frame.transform.rotation.z = rotation[2]
                        tf_odom_marker_frame.transform.rotation.w = rotation[3]
                        tf_transformer.setTransform(tf_odom_marker_frame)

                        ''' set transform marker frame -> table root '''
                        tf_marker_frame_table_root = geometry_msgs.TransformStamped()
                        tf_marker_frame_table_root.header.stamp = tf_time
                        tf_marker_frame_table_root.header.frame_id = marker.header.frame_id
                        tf_marker_frame_table_root.child_frame_id = "table_root"
                        tf_marker_frame_table_root.transform.translation = marker.pose.position
                        tf_marker_frame_table_root.transform.rotation = marker.pose.orientation
                        tf_transformer.setTransform(tf_marker_frame_table_root)
                        table_root_trans = tf_transformer.lookupTransform(map_root,
                                                                          tf_marker_frame_table_root.child_frame_id,
                                                                          rospy.Time(0))

                        '''set transform table root -> table front left'''
                        tf_table_front_left = geometry_msgs.TransformStamped()
                        tf_table_front_left.header.stamp = tf_time
                        tf_table_front_left.header.frame_id = "table_root"
                        tf_table_front_left.child_frame_id = "table_front_left"
                        tf_table_front_left.transform.translation = marker.points[0]
                        tf_table_front_left.transform.rotation.w = 1.0
                        tf_transformer.setTransform(tf_table_front_left)
                        front_left_trans = tf_transformer.lookupTransform(map_root, "table_front_left", rospy.Time(0))

                        '''set transform table root -> table back right'''
                        tf_table_front_left = geometry_msgs.TransformStamped()
                        tf_table_front_left.header.stamp = tf_time
                        tf_table_front_left.header.frame_id = "table_root"
                        tf_table_front_left.child_frame_id = "table_back_right"
                        tf_table_front_left.transform.translation = marker.points[2]
                        tf_table_front_left.transform.rotation.w = 1.0
                        tf_transformer.setTransform(tf_table_front_left)
                        back_right_trans = tf_transformer.lookupTransform(map_root, "table_back_right", rospy.Time(0))

                        ''' determine table dimensions '''
                        table_corners = tf_transformer.lookupTransform("table_front_left",
                                                                       "table_back_right",
                                                                       rospy.Time(0))
                        width = math.sqrt(math.pow(table_corners[0][0], 2))
                        length = math.sqrt(math.pow(table_corners[0][1], 2))
                        height = front_left_trans[0][2]

                        ''' set table centre '''
                        tf_table_centre = geometry_msgs.TransformStamped()
                        tf_table_centre.header.stamp = tf_time
                        tf_table_centre.header.frame_id = "table_front_left"
                        tf_table_centre.child_frame_id = "table_centre"
                        tf_table_centre.transform.translation.x = width * 0.5
                        tf_table_centre.transform.translation.y = length * 0.5
                        tf_table_centre.transform.translation.z = 0.0
                        tf_table_centre.transform.rotation.w = 1.0
                        tf_transformer.setTransform(tf_table_centre)
                        table_centre_trans = tf_transformer.lookupTransform(map_root, "table_centre", rospy.Time(0))

                        ''' set table front '''
                        tf_table_front_centre = geometry_msgs.TransformStamped()
                        tf_table_front_centre.header.stamp = tf_time
                        tf_table_front_centre.header.frame_id = "table_front_left"
                        tf_table_front_centre.child_frame_id = "table_front_centre"
                        tf_table_front_centre.transform.translation.x = width * 0.5
                        tf_table_front_centre.transform.translation.y = 0.0
                        tf_table_front_centre.transform.translation.z = 0.0
                        tf_table_front_centre.transform.rotation.w = 1.0
                        tf_transformer.setTransform(tf_table_front_centre)

                        ''' determine front place pose '''
                        table_centre_front_centre_trans = tf_transformer.lookupTransform("table_front_centre",
                                                                                         "table_centre",
                                                                                         rospy.Time(0))
                        print 'table front to centre:'
                        print table_centre_front_centre_trans

                        ''' determine table pose '''
                        table_pos_x = table_centre_trans[0][0]
                        table_pos_y = table_centre_trans[0][1]
                        table_pos_z = table_centre_trans[0][2]
                        rospy.loginfo("New table's properties:")
                        rospy.loginfo("x = " + str(table_pos_x) + ", y = " + str(table_pos_y)
                                      + ", z = " + str(table_pos_z))
                        rospy.loginfo("width = " + str(width) + ", length = " + str(length)
                                      + ", height = " + str(height))

                        # prepare collision object
                        collision_object = moveit_msgs.CollisionObject()
                        collision_object.header.stamp = tf_time
                        collision_object.header.frame_id = map_root
#                        collision_object.id = "table_" + str(rospy.Time.now())
                        collision_object.id = "table"
    #                    collision_object.type = 
                        object_shape = shape_msgs.SolidPrimitive()
                        object_shape.type = shape_msgs.SolidPrimitive.BOX
                        object_shape.dimensions.append(width + self._table_width_enlargement) # BOX_X
                        object_shape.dimensions.append(length + self._table_length_enlargement) # BOX_Y
                        object_shape.dimensions.append(height + self._table_height_enlargement) # BOX_Z
                        collision_object.primitives.append(object_shape)
                        shape_pose = geometry_msgs.Pose()
                        shape_pose.position.x = table_pos_x
                        shape_pose.position.y = table_pos_y
                        shape_pose.position.z = 0.0 + height / 2 # reference is at the centre of the mesh
                        shape_pose.orientation.x = front_left_trans[1][0]
                        shape_pose.orientation.y = front_left_trans[1][1]
                        shape_pose.orientation.z = front_left_trans[1][2]
                        shape_pose.orientation.w = front_left_trans[1][3]
                        collision_object.primitive_poses.append(shape_pose)
                        collision_object.operation = moveit_msgs.CollisionObject.ADD
                        self._pub_collision_object.publish(collision_object)
                        rospy.loginfo('Table collision object published.')
                        userdata.tabletop_centre_pose.header.stamp = tf_time
                        userdata.tabletop_centre_pose.header.frame_id = map_root
                        userdata.tabletop_centre_pose.pose.position.x = table_pos_x
                        userdata.tabletop_centre_pose.pose.position.y = table_pos_y
                        userdata.tabletop_centre_pose.pose.position.z = table_pos_z
                        userdata.tabletop_centre_pose.pose.orientation.x = front_left_trans[1][0]
                        userdata.tabletop_centre_pose.pose.orientation.y = front_left_trans[1][1]
                        userdata.tabletop_centre_pose.pose.orientation.z = front_left_trans[1][2]
                        userdata.tabletop_centre_pose.pose.orientation.w = front_left_trans[1][3]
                        userdata.tabletop_front_place_pose.header.stamp = tf_time
                        userdata.tabletop_front_place_pose.header.frame_id = map_root
                        angle = math.atan2(table_centre_front_centre_trans[0][1], table_centre_front_centre_trans[0][0])
                        dist = math.sqrt(math.pow(table_centre_front_centre_trans[0][0], 2)\
                                          + math.pow(table_centre_front_centre_trans[0][1], 2))
                        userdata.tabletop_front_place_pose.pose.position.x = table_pos_x - dist * math.sin(angle) * self._front_place_pose_factor
                        userdata.tabletop_front_place_pose.pose.position.y = table_pos_y - dist * math.cos(angle) * self._front_place_pose_factor
                        userdata.tabletop_front_place_pose.pose.position.z = table_pos_z + table_centre_front_centre_trans[0][2]
                        userdata.tabletop_front_place_pose.pose.orientation.x = table_centre_front_centre_trans[1][0]
                        userdata.tabletop_front_place_pose.pose.orientation.y = table_centre_front_centre_trans[1][1]
                        userdata.tabletop_front_place_pose.pose.orientation.z = table_centre_front_centre_trans[1][2]
                        userdata.tabletop_front_place_pose.pose.orientation.w = table_centre_front_centre_trans[1][3]
                        rospy.loginfo('Tabletop centre pose:')
                        rospy.loginfo(userdata.tabletop_centre_pose)
                        rospy.loginfo('Tabletop front place pose:')
                        rospy.loginfo(userdata.tabletop_front_place_pose)
                        table_added = True
                    except tf.Exception as e:
                        rospy.logwarn("TF reported an error: " + str(e))
                        pass
                    except Exception as e:
                        rospy.logwarn("Exception occurred: " + str(e))
                        pass
                markers_processed = True
            else:
                rospy.loginfo("Waiting for incoming table message ...")
                rate.sleep()
        if table_added:
            duration = 2.0
            rospy.loginfo('Waiting for ' + str(duration) + ' seconds.')
            rospy.sleep(duration) # wait a bit to make sure all subscribers will receive the message
            rospy.loginfo("Table added to planning scene.");
            return 'table_added'
        else:
            return 'no_table_added'
