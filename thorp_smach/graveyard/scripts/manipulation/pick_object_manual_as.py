#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('thorp_smach')
import thorp_smach
from thorp_smach.state_machines import pick_object_sm
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from smach_ros import ActionServerWrapper
import tf

import geometry_msgs.msg as geometry_msgs
import manipulation_msgs.msg as manipulation_msgs
import moveit_msgs.msg as moveit_msgs
import pick_and_place_msgs.msg as pick_and_place_msgs
import sensor_msgs.msg as sensor_msgs


class ParseGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['parsed'],
                             input_keys=['goal'],
                             output_keys=['object_name',
                                          'object_pose',
                                          'feedback'])

    def execute(self, userdata):
        rospy.loginfo('Parsing pick object goal ...')
        userdata.object_name = userdata.goal.object_name
        object_pose = geometry_msgs.PoseStamped()
        object_pose = userdata.goal.object_pose
        userdata.object_pose = object_pose
        feedback = pick_and_place_msgs.PickObjectFeedback()
        feedback.process_state = 'PickObjectGoal parsed'
        userdata.feedback = feedback
        rospy.loginfo('Gripper goal prepared.')
        return 'parsed'

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

def main():
    rospy.init_node('pick_object')
    
    sm = smach.StateMachine(outcomes=['picked',
                                      'pick_failed',
                                      'preempted',
                                      'error'],
                            input_keys=['goal'],
                            output_keys=['feedback',
                                         'result'])

    with sm:
        sm.userdata.result = pick_and_place_msgs.PickObjectResult()
        sm.userdata.pose_arm_default = geometry_msgs.PoseStamped()
        sm.userdata.pose_arm_default.header.stamp = rospy.Time.now()
        sm.userdata.pose_arm_default.header.frame_id = "/base_footprint"
        sm.userdata.pose_arm_default.pose.position.x = 0.1
        sm.userdata.pose_arm_default.pose.position.y = 0.0
        sm.userdata.pose_arm_default.pose.position.z = 0.5
        sm.userdata.pose_arm_default.pose.orientation.w = 1.0
        sm.userdata.pre_grasp_dist = 0.27
        sm.userdata.pre_grasp_height = 0.10
        sm.userdata.grasp_dist = 0.15
        sm.userdata.post_grasp_height = 0.20
        sm.userdata.angle_gripper_opened = 1.3
        sm.userdata.angle_gripper_closed = 0.2
#        sm.userdata.object_names = list()
        
        smach.StateMachine.add('ParseGoal',
                               ParseGoal(),
                               remapping={'goal':'goal',
                                          'feedback':'feedback',
                                          'object_pose':'object_pose'},
                               transitions={'parsed':'PreparePickObject'})
        
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
        
        sm_pick = pick_object_manual_sm.createSM()
        smach.StateMachine.add('PickObject',
                               sm_pick,
                               remapping={'object_name':'pick_object_name',
                                          'object_pose':'pick_object_pose',
                                          'collision_object':'pick_collision_object',
                                          'pre_grasp_dist':'pre_grasp_dist',
                                          'pre_grasp_height':'pre_grasp_height',
                                          'grasp_dist':'grasp_dist',
                                          'post_grasp_height':'post_grasp_height',
                                          'angle_gripper_open':'angle_gripper_opened',
                                          'angle_gripper_closed':'angle_gripper_closed',
                                          'pose_arm_default':'pose_arm_default'},
                               transitions={'picked':'picked',
                                            'pick_failed':'pick_failed',
                                            'preempted':'preempted'})
    
    asw = ActionServerWrapper('pick_object',
                              pick_and_place_msgs.PickObjectAction,
                              wrapped_container = sm,
                              goal_key = 'goal',
                              feedback_key = 'feedback',
                              result_key = 'result',
                              succeeded_outcomes = ['picked'],
                              aborted_outcomes = ['pick_failed'],
                              preempted_outcomes = ['preempted'])
    
    asw.run_server()
    
    rospy.spin()

    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
