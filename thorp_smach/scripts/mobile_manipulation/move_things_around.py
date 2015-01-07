#!/usr/bin/env python

'''
Move Things Around App

App for picking objects up from a table in one place and placing them on a table in another place
'''

import math
import tf
from thorp_smach.state_machines.state_machines_imports import *
from thorp_smach.state_machines import find_object_sm, \
                                       find_table_sm, \
                                       pick_object_sm, \
                                       pick_object_manual_sm, \
                                       place_object_sm, \
                                       place_object_manual_sm
from thorp_smach.pick_and_place_tools.msg_imports import *
from thorp_smach.pick_and_place_tools import misc_tools
import move_base_msgs.msg as move_base_msgs


class Prepare(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['prepared'],
                             input_keys=[],
                             output_keys=[])
    def execute(self, userdata):
        return 'prepared'

class PreparePickObject(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['prepared',
                                       'error'],
                             input_keys=['recognised_objects',
                                         'recognised_object_names',
                                         'objects_info',
                                         'pick_object_pose',
                                         'pick_object_name',
                                         'tf_listener'],
                             output_keys=['pick_object_pose',
                                         'pick_object_name',
                                         'pick_collision_object',
                                         'tf_listener'])
    def execute(self, userdata):
        max_confidence = 0.0
        object_index = 0
        if len(userdata.recognised_objects.objects) is not len(userdata.recognised_object_names):
            rospy.logerr("Number of recognised objects does not match object names!")
            return 'error'
        for object in userdata.recognised_objects.objects:
            if object.confidence > max_confidence:
                max_confidence = object.confidence
                object_index = userdata.recognised_objects.objects.index(object)

        userdata.pick_object_name = userdata.recognised_object_names[object_index]
        userdata.pick_object_pose = geometry_msgs.PoseStamped()
        userdata.pick_object_pose.header = userdata.recognised_objects.objects[object_index].pose.header
        userdata.pick_object_pose.pose = userdata.recognised_objects.objects[object_index].pose.pose.pose

        pick_collision_object = moveit_msgs.CollisionObject()
        pick_collision_object.header = userdata.pick_object_pose.header
        pick_collision_object.id = userdata.pick_object_name
        pick_collision_object.type = userdata.recognised_objects.objects[object_index].type
        shape = shape_msgs.SolidPrimitive()
        shape.type = shape_msgs.SolidPrimitive.CYLINDER
        shape.dimensions.append(0.20) # CYLINDER_HEIGHT
        shape.dimensions.append(0.05) # CYLINDER_RADIUS
        shape_pose = geometry_msgs.Pose()
        shape_pose.position.x = 0.15
        quat = tf.transformations.quaternion_from_euler(-math.pi / 2, 0.0, 0.0)
        shape_pose.orientation = geometry_msgs.Quaternion(*quat)
        pick_collision_object.primitives.append(shape)
        pick_collision_object.primitive_poses.append(shape_pose)
        pick_collision_object.meshes.append(userdata.objects_info[object_index].information.ground_truth_mesh)
        pick_collision_object.mesh_poses.append(userdata.pick_object_pose.pose)
        userdata.pick_collision_object = pick_collision_object
        rospy.loginfo("Collision object:")
        rospy.loginfo(pick_collision_object.header)
        rospy.loginfo(pick_collision_object.primitive_poses)
        rospy.loginfo(pick_collision_object.mesh_poses)

        rospy.loginfo("Object '" + str(userdata.pick_object_name) + "' has been selected among all recognised objects")
        rospy.loginfo("Object's pose:")
        rospy.loginfo(userdata.pick_object_pose)

        angle = math.atan2(userdata.pick_object_pose.pose.position.y, userdata.pick_object_pose.pose.position.x)
        dist = math.sqrt(math.pow(userdata.pick_object_pose.pose.position.x, 2) + math.pow(userdata.pick_object_pose.pose.position.y, 2))
        userdata.pick_object_pose.pose.position.x = dist * math.cos(angle)
        userdata.pick_object_pose.pose.position.y = dist * math.sin(angle)
        userdata.pick_object_pose.pose.position.z = userdata.pick_object_pose.pose.position.z
        yaw = angle
        roll = math.pi / 2
        pitch = 0.0
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        userdata.pick_object_pose.pose.orientation = geometry_msgs.Quaternion(*quat)
#        rospy.loginfo("Object's pose after adaption:")
#        rospy.loginfo(userdata.pick_object_pose)
        return 'prepared'

class Finalise(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finalised'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        return 'finalised'


def main():
    rospy.init_node('move_things_around')

    sm = StateMachine(outcomes=['success',
                                'aborted',
                                'preempted'])
    with sm:
        ''' general '''
        sm.userdata.true = True
        sm.userdata.false = False
        ''' table poses '''
        sm.userdata.pose_table_a = geometry_msgs.PoseStamped()
        sm.userdata.pose_table_a.header.stamp = rospy.Time.now()
        sm.userdata.pose_table_a.header.frame_id = "map"
        sm.userdata.pose_table_a.pose.position.x = -1.0
        sm.userdata.pose_table_a.pose.position.y = 4.0
        sm.userdata.pose_table_a.pose.orientation.x = 0.0
        sm.userdata.pose_table_a.pose.orientation.y = 0.0
        sm.userdata.pose_table_a.pose.orientation.z = 0.851
        sm.userdata.pose_table_a.pose.orientation.w = 0.526
        sm.userdata.pose_table_b = geometry_msgs.PoseStamped()
        sm.userdata.pose_table_b.header = sm.userdata.pose_table_a.header
        sm.userdata.pose_table_b.pose.position.x = 0.95
        sm.userdata.pose_table_b.pose.position.y = 0.75
        sm.userdata.pose_table_b.pose.orientation.x = 0.0
        sm.userdata.pose_table_b.pose.orientation.y = 0.0
        sm.userdata.pose_table_b.pose.orientation.z = -0.509
        sm.userdata.pose_table_b.pose.orientation.w = 0.861
        ''' Korus base pose '''
        sm.userdata.base_position = geometry_msgs.PoseStamped()
        ''' tabletop poses '''
        sm.userdata.pose_tabletop_a = geometry_msgs.PoseStamped()
        sm.userdata.pose_tabletop_a.header.stamp = rospy.Time.now()
        sm.userdata.pose_tabletop_a.header.frame_id = "base_footprint"
        sm.userdata.pose_tabletop_a.pose.position.x = 0.6
        sm.userdata.pose_tabletop_a.pose.position.y = 0.0
        sm.userdata.pose_tabletop_a.pose.position.z = 0.55
        sm.userdata.pose_tabletop_a.pose.orientation.x = 0.0
        sm.userdata.pose_tabletop_a.pose.orientation.y = 0.0
        sm.userdata.pose_tabletop_a.pose.orientation.z = 0.0
        sm.userdata.pose_tabletop_a.pose.orientation.w = 1.0
        sm.userdata.pose_tabletop_b = geometry_msgs.PoseStamped()
        sm.userdata.pose_tabletop_b.header = sm.userdata.pose_tabletop_a.header
        sm.userdata.pose_tabletop_b.pose.position.x = 1.0
        sm.userdata.pose_tabletop_b.pose.position.y = 0.0
        sm.userdata.pose_tabletop_b.pose.position.z = 0.55
        sm.userdata.pose_tabletop_b.pose.orientation = sm.userdata.pose_tabletop_a.pose.orientation
        ''' find object'''
        sm.userdata.min_confidence = 0.8
        sm.userdata.object_names = list()
        sm.userdata.tf_listener = tf.TransformListener()
        ''' pick object'''
        sm.userdata.pre_grasp_dist = 0.27
        sm.userdata.pre_grasp_height = 0.12
        sm.userdata.grasp_dist = 0.14
        sm.userdata.grasp_height = 0.08
        sm.userdata.post_grasp_dist = sm.userdata.pre_grasp_dist
        sm.userdata.post_grasp_height = 0.23
        sm.userdata.angle_gripper_opened = 1.3
        sm.userdata.angle_gripper_closed = 0.2
        sm.userdata.pose_arm_default = geometry_msgs.PoseStamped()
        sm.userdata.pose_arm_default.header = sm.userdata.pose_tabletop_a.header
        sm.userdata.pose_arm_default.pose.position.x = 0.1
        sm.userdata.pose_arm_default.pose.position.y = 0.0
        sm.userdata.pose_arm_default.pose.position.z = 0.5
        sm.userdata.pose_arm_default.pose.orientation = sm.userdata.pose_tabletop_a.pose.orientation
        sm.userdata.min_confidence = 0.8
        sm.userdata.object_names = list()
        ''' place object'''
        sm.userdata.centre_place_pose = geometry_msgs.PoseStamped()
        sm.userdata.front_place_pose = geometry_msgs.PoseStamped()
        sm.userdata.pre_place_dist = 0.20
        sm.userdata.pre_place_height = 0.20
        sm.userdata.place_dist = sm.userdata.grasp_dist
        sm.userdata.place_height = sm.userdata.grasp_height + 0.02
        sm.userdata.post_place_dist = 0.29
        sm.userdata.post_place_height = 0.20

        smach.StateMachine.add('Prepare',
                               Prepare(),
                               remapping={},
                               transitions={'prepared':'ClearCollisionObjects'})

        smach.StateMachine.add('ClearCollisionObjects',
                               misc_tools.ClearCollisionObjects(),
                               remapping={},
                               transitions={'cleared':'MoveToTableA',
                                            'clearing_failed':'aborted'})

        smach.StateMachine.add('MoveToTableA',
                               SimpleActionState('move_base',
                                                 move_base_msgs.MoveBaseAction,
                                                 goal_slots=['target_pose'],
                                                 result_slots=[]),
                               remapping={'target_pose':'pose_table_a',
                                          'base_position':'base_position'},
                               transitions={'succeeded':'FindObject',
                                            'aborted':'aborted',
                                            'preempted':'preempted'})

        sm_find_object = find_object_sm.createSM()
        smach.StateMachine.add('FindObject',
                               sm_find_object,
                               remapping={'table_pose':'pose_tabletop_a',
                                          'look_around':'false',
                                          'min_confidence':'min_confidence',
                                          'recognised_objects':'recognised_objects',
                                          'object_names':'object_names',
                                          'object_pose':'pick_object_pose',
                                          'objects_info':'objects_info',
                                          'collision_object':'collision_object',
                                          'error_message':'error_message',
                                          'error_code':'error_code',
                                          'tf_listener':'tf_listener'},
                               transitions={'object_found':'PreparePickObject',
                                            'no_objects_found':'Prepare',
                                            'aborted':'aborted',
                                            'preempted':'preempted'})

        smach.StateMachine.add('PreparePickObject',
                               PreparePickObject(),
                               remapping={'recognised_objects':'recognised_objects',
                                          'recognised_object_names':'object_names',
                                          'objects_info':'objects_info',
                                          'pick_object_pose':'pick_object_pose',
                                          'pick_object_name':'pick_object_name',
                                          'pick_collision_object':'pick_collision_object',
                                          'tf_listener':'tf_listener'},
                               transitions={'prepared':'PickObject',
                                            'error':'aborted'})

#        sm_pick_object = pick_object_sm.createSM() # using MoveIt's pick action
        sm_pick_object = pick_object_manual_sm.createSM() # using own pick implementation
        smach.StateMachine.add('PickObject',
                               sm_pick_object,
                               remapping={'object_name':'pick_object_name',
                                          'object_pose':'pick_object_pose',
                                          'collision_object':'pick_collision_object',
                                          'pre_grasp_dist':'pre_grasp_dist',
                                          'pre_grasp_height':'pre_grasp_height',
                                          'grasp_dist':'grasp_dist',
                                          'grasp_height':'grasp_height',
                                          'post_grasp_dist':'post_grasp_dist',
                                          'post_grasp_height':'post_grasp_height',
                                          'angle_gripper_open':'angle_gripper_opened',
                                          'angle_gripper_closed':'angle_gripper_closed',
                                          'pose_arm_default':'pose_arm_default'},
                               transitions={'picked':'MoveToTableB',
                                            'pick_failed':'Prepare',
                                            'preempted':'preempted'})

        smach.StateMachine.add('MoveToTableB',
                               SimpleActionState('move_base',
                                                 move_base_msgs.MoveBaseAction,
                                                 goal_slots=['target_pose'],
                                                 result_slots=[]),
                               remapping={'target_pose':'pose_table_b',
                                          'base_position':'base_position'},
                               transitions={'succeeded':'FindTable',
                                            'aborted':'aborted',
                                            'preempted':'preempted'})

        sm_find_table = find_table_sm.createSM()
        smach.StateMachine.add('FindTable',
                               sm_find_table,
                               remapping={'table_pose':'pose_tabletop_b',
                                          'tabletop_centre_pose':'centre_place_pose',
                                          'tabletop_front_place_pose':'front_place_pose',
                                          'look_around':'false',
                                          'error_message':'error_message',
                                          'error_code':'error_code',
                                          'tf_listener':'tf_listener'},
                               transitions={'table_found':'PlaceObject',
                                            'no_table_found':'aborted',
                                            'aborted':'aborted',
                                            'preempted':'preempted'})

#        sm_place_object = place_object_sm.createSM() # using MoveIt's place action
        sm_place_object = place_object_manual_sm.createSM() # using own place implementation
        smach.StateMachine.add('PlaceObject',
                               sm_place_object,
                               remapping={'place_pose':'front_place_pose',
                                          'pre_place_dist':'pre_place_dist',
                                          'pre_place_height':'pre_place_height',
                                          'place_dist':'place_dist',
                                          'place_height':'place_height',
                                          'post_place_dist':'post_place_dist',
                                          'post_place_height':'post_place_height',
                                          'collision_object':'pick_collision_object',
                                          'angle_gripper_opened':'angle_gripper_opened',
                                          'angle_gripper_closed':'angle_gripper_closed',
                                          'pose_arm_default':'pose_arm_default'},
                               transitions={'placed':'Prepare',
                                            'place_failed':'Finalise',
                                            'preempted':'preempted'})

        smach.StateMachine.add('Finalise',
                               Finalise(),
                               remapping={},
                               transitions={'finalised':'success'})

    sm.execute()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

