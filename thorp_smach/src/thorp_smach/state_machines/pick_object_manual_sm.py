#!/usr/bin/env python
import math
import tf
from thorp_smach.state_machines.state_machines_imports import *
from thorp_smach.pick_and_place_tools.msg_imports import *
from thorp_smach.pick_and_place_tools import trajectory_control, misc_tools


class Prepare(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['prepared'],
                             input_keys=['object_pose',
                                         'pre_grasp_pose',
                                         'grasp_pose',
                                         'post_grasp_pose',
                                         'pre_grasp_dist',
                                         'pre_grasp_height',
                                         'grasp_dist',
                                         'grasp_height',
                                         'post_grasp_dist',
                                         'post_grasp_height',
                                         'collision_object'],
                             output_keys=['pre_grasp_pose',
                                          'grasp_pose',
                                          'post_grasp_pose',
                                          'collision_object'])
    def execute(self, userdata):
        userdata.pre_grasp_pose.header.stamp = rospy.Time.now()
        userdata.pre_grasp_pose.header.frame_id = "/base_footprint"
        userdata.grasp_pose.header = userdata.pre_grasp_pose.header
        userdata.post_grasp_pose.header = userdata.pre_grasp_pose.header

        angle = math.atan2(userdata.object_pose.pose.position.y, userdata.object_pose.pose.position.x)
        dist = math.sqrt(math.pow(userdata.object_pose.pose.position.x, 2)
                         + math.pow(userdata.object_pose.pose.position.y, 2))
        yaw = angle
        roll = math.pi / 2
        pitch = 0.0
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pick_height = userdata.object_pose.pose.position.z
        userdata.pre_grasp_pose.pose.position.x = (dist - userdata.pre_grasp_dist) * math.cos(angle)
        userdata.pre_grasp_pose.pose.position.y = (dist - userdata.pre_grasp_dist) * math.sin(angle)
        userdata.pre_grasp_pose.pose.position.z = pick_height + userdata.pre_grasp_height
        userdata.pre_grasp_pose.pose.orientation = geometry_msgs.Quaternion(*quat)
        userdata.grasp_pose.pose.position.x = (dist - userdata.grasp_dist) * math.cos(angle)
        userdata.grasp_pose.pose.position.y = (dist - userdata.grasp_dist) * math.sin(angle)
        userdata.grasp_pose.pose.position.z = pick_height + userdata.grasp_height
        userdata.grasp_pose.pose.orientation = geometry_msgs.Quaternion(*quat)
        userdata.post_grasp_pose.pose.position.x = (dist - userdata.post_grasp_dist) * math.cos(angle)
        userdata.post_grasp_pose.pose.position.y = (dist - userdata.post_grasp_dist) * math.sin(angle)
        userdata.post_grasp_pose.pose.position.z = pick_height + userdata.post_grasp_height
        userdata.post_grasp_pose.pose.orientation = geometry_msgs.Quaternion(*quat)

        rospy.loginfo('Object pose:')
        rospy.loginfo(userdata.object_pose.pose)
        rospy.loginfo('Pre grasp pose:')
        rospy.loginfo(userdata.pre_grasp_pose.pose)
        rospy.loginfo('Grasp pose:')
        rospy.loginfo(userdata.grasp_pose.pose)
        rospy.loginfo('Post grasp pose:')
        rospy.loginfo(userdata.post_grasp_pose.pose)

        userdata.collision_object.header.stamp = rospy.Time.now()
        userdata.collision_object.header.frame_id = "palm_link"
        new_pose = geometry_msgs.Pose()
        new_pose.position.x = 0.15
        new_pose.position.y = -0.05
        roll = -math.pi / 2
        quat = tf.transformations.quaternion_from_euler(roll, 0.0, 0.0)
        new_pose.orientation = geometry_msgs.Quaternion(*quat)
        userdata.collision_object.mesh_poses[0] = new_pose
        return 'prepared'


def createSM():
    sm = smach.StateMachine(outcomes=['picked',
                                      'pick_failed',
                                      'preempted'],
                            input_keys=['object_name',
                                        'object_pose',
                                        'collision_object',
                                        'pre_grasp_dist',
                                        'pre_grasp_height',
                                        'grasp_dist',
                                        'grasp_height',
                                        'post_grasp_dist',
                                        'post_grasp_height',
                                        'angle_gripper_opened',
                                        'angle_gripper_closed',
                                        'pose_arm_default'],
                            output_keys=['object_pose'])

    with sm:
        sm.userdata.pre_grasp_pose = geometry_msgs.PoseStamped()
        sm.userdata.grasp_pose = geometry_msgs.PoseStamped()
        sm.userdata.post_grasp_pose = geometry_msgs.PoseStamped()
        sm.userdata.true = True
        sm.userdata.false = False
        sm.userdata.wait_5sec = 5.0
        sm.userdata.move_arm_up_distance = 0.03

        smach.StateMachine.add('Prepare',
                               Prepare(),
                               remapping={'object_pose':'object_pose',
                                          'pre_grasp_pose':'pre_grasp_pose',
                                          'grasp_pose':'grasp_pose',
                                          'post_grasp_pose':'post_grasp_pose',
                                          'pre_grasp_dist':'pre_grasp_dist',
                                          'pre_grasp_height':'pre_grasp_height',
                                          'grasp_dist':'grasp_dist',
                                          'grasp_height':'grasp_height',
                                          'post_grasp_dist':'post_grasp_dist',
                                          'post_grasp_height':'post_grasp_height'},
                               transitions={'prepared':'MoveArmPreGrasp'})

        smach.StateMachine.add('MoveArmPreGrasp',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pre_grasp_pose'},
                               transitions={'succeeded':'OpenGripper',
                                            'aborted':'MoveArmDefaultFailed',
                                            'preempted':'preempted'})

        smach.StateMachine.add('OpenGripper',
                               SimpleActionState('gripper_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.gripperControlGoalCb,
                                                 result_cb=trajectory_control.gripperControlResultCb),
                               remapping={'angle':'angle_gripper_opened',
                                          'open_gripper':'true',
                                          'close_gripper':'false'},
                               transitions={'succeeded':'MoveArmIKGrasp',
                                            'aborted':'MoveArmDefaultFailed',
                                            'preempted':'preempted'})

        smach.StateMachine.add('MoveArmIKGrasp',
                               SimpleActionState('move_arm_ik',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'grasp_pose'},
                               transitions={'succeeded':'CloseGripper',
                                            'aborted':'MoveArmDefaultFailed',
                                            'preempted':'preempted'})

        smach.StateMachine.add('CloseGripper',
                               SimpleActionState('gripper_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.gripperControlGoalCb,
                                                 result_cb=trajectory_control.gripperControlResultCb),
                               remapping={'angle':'angle_gripper_closed',
                                          'open_gripper':'false',
                                          'close_gripper':'true'},
                               transitions={'succeeded':'AttachObject',
                                            'aborted':'MoveArmDefaultFailed',
                                            'preempted':'preempted'})

        smach.StateMachine.add('AttachObject',
                               misc_tools.AttachObjectToRobot(),
                               remapping={'collision_object':'collision_object',
                                          'attach':'true'},
                               transitions={'done':'WaitForObjectAdded'})

        smach.StateMachine.add('WaitForObjectAdded', misc_tools.Wait(),
                               remapping={'duration':'wait_5sec'},
                               transitions={'done':'RetrieveJointStates'})

        smach.StateMachine.add('RetrieveJointStates',
                               misc_tools.RetrieveJointStates(),
                               remapping={'joint_states':'joint_states'},
                               transitions={'success':'MoveArmUp',
                                            'error':'MoveArmIKPostGrasp'})

        smach.StateMachine.add('MoveArmUp',
                               SimpleActionState('arm_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.moveArmUpGoalCb,
                                                 result_cb=trajectory_control.generalResponseCb),
                               remapping={'joint_states':'joint_states',
                                          'distance':'move_arm_up_distance'},
                               transitions={'succeeded':'MoveArmIKPostGrasp',
                                            'aborted':'MoveArmIKPostGrasp',
                                            'preempted':'preempted'})

        smach.StateMachine.add('MoveArmIKPostGrasp',
                               SimpleActionState('move_arm_ik',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'post_grasp_pose'},
                               transitions={'succeeded':'MoveArmDefault',
                                            'aborted':'MoveArmDefaultFailed',
                                            'preempted':'preempted'})

        smach.StateMachine.add('MoveArmDefault',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pose_arm_default'},
                               transitions={'succeeded':'picked',
                                            'aborted':'pick_failed',
                                            'preempted':'preempted'})

        smach.StateMachine.add('MoveArmDefaultFailed',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pose_arm_default'},
                               transitions={'succeeded':'pick_failed',
                                            'aborted':'pick_failed',
                                            'preempted':'preempted'})
    return sm
