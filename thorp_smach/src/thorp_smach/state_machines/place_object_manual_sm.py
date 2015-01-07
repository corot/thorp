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
                             input_keys=['pre_place_pose',
                                         'place_pose',
                                         'post_place_pose',
                                         'pre_place_dist',
                                         'pre_place_height',
                                         'place_dist',
                                         'place_height',
                                         'post_place_dist',
                                         'post_place_height',
                                         'collision_object'],
                             output_keys=['pre_place_pose',
                                          'place_pose',
                                          'post_place_pose',
                                          'collision_object'])
    def execute(self, userdata):
        userdata.pre_place_pose.header = userdata.place_pose.header
        userdata.post_place_pose.header = userdata.place_pose.header

        angle = math.atan2(userdata.place_pose.pose.position.y, userdata.place_pose.pose.position.x)
        dist = math.sqrt(math.pow(userdata.place_pose.pose.position.x, 2)
                         + math.pow(userdata.place_pose.pose.position.y, 2))
        yaw = angle
        roll = math.pi / 2
        pitch = 0.0
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        place_heigth = userdata.place_pose.pose.position.z
        userdata.pre_place_pose.pose.position.x = (dist - userdata.pre_place_dist) * math.cos(angle)
        userdata.pre_place_pose.pose.position.y = (dist - userdata.pre_place_dist) * math.sin(angle)
        userdata.pre_place_pose.pose.position.z = place_heigth + userdata.pre_place_height
        userdata.pre_place_pose.pose.orientation = geometry_msgs.Quaternion(*quat)
        userdata.place_pose.pose.position.x = (dist - userdata.place_dist) * math.cos(angle)
        userdata.place_pose.pose.position.y = (dist - userdata.place_dist) * math.sin(angle)
        userdata.place_pose.pose.position.z = place_heigth + userdata.place_height
        userdata.place_pose.pose.orientation = geometry_msgs.Quaternion(*quat)
        userdata.post_place_pose.pose.position.x = (dist - userdata.post_place_dist) * math.cos(angle)
        userdata.post_place_pose.pose.position.y = (dist - userdata.post_place_dist) * math.sin(angle)
        userdata.post_place_pose.pose.position.z = place_heigth + userdata.post_place_height
        userdata.post_place_pose.pose.orientation = geometry_msgs.Quaternion(*quat)

        userdata.collision_object.header = userdata.place_pose.header
        userdata.collision_object.mesh_poses[0] = userdata.place_pose.pose

        rospy.loginfo('Pre place pose:')
        rospy.loginfo(userdata.pre_place_pose)
        rospy.loginfo('Place pose:')
        rospy.loginfo(userdata.place_pose)
        rospy.loginfo('Post place pose:')
        rospy.loginfo(userdata.post_place_pose)
        return 'prepared'


def createSM():

    sm = smach.StateMachine(outcomes=['placed',
                                      'place_failed',
                                      'preempted'],
                            input_keys=['place_pose',
                                        'pre_place_dist',
                                        'pre_place_height',
                                        'place_dist',
                                        'place_height',
                                        'post_place_dist',
                                        'post_place_height',
                                        'collision_object',
                                        'angle_gripper_opened',
                                        'angle_gripper_closed',
                                        'pose_arm_default'],
                            output_keys=[])

    with sm:
        sm.userdata.pre_place_pose = geometry_msgs.PoseStamped()
        sm.userdata.place_pose = geometry_msgs.PoseStamped()
        sm.userdata.post_place_pose = geometry_msgs.PoseStamped()
        sm.userdata.true = True
        sm.userdata.false = False
        sm.userdata.wait_2sec = 2.0
        sm.userdata.wait_4sec = 4.0
        sm.userdata.move_arm_down_distance = -0.03

        smach.StateMachine.add('Prepare',
                               Prepare(),
                               remapping={'pre_place_pose':'pre_place_pose',
                                          'place_pose':'place_pose',
                                          'post_place_pose':'post_place_pose',
                                          'pre_place_dist':'pre_place_dist',
                                          'pre_place_height':'pre_place_height',
                                          'place_dist':'place_dist',
                                          'place_height':'place_height',
                                          'post_place_dist':'post_place_dist',
                                          'post_place_height':'post_place_height',
                                          'collision_object':'collision_object'},
                               transitions={'prepared':'MoveArmPrePlace'})

        smach.StateMachine.add('MoveArmPrePlace',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pre_place_pose'},
                               transitions={'succeeded':'MoveArmIKPlace',
                                            'aborted':'place_failed',
                                            'preempted':'preempted'})

        smach.StateMachine.add('MoveArmIKPlace',
                               SimpleActionState('move_arm_ik',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'place_pose'},
                               transitions={'succeeded':'RetrieveJointStates',
                                            'aborted':'MoveArmDefault',
                                            'preempted':'preempted'})

        smach.StateMachine.add('RetrieveJointStates',
                               misc_tools.RetrieveJointStates(),
                               remapping={'joint_states':'joint_states'},
                               transitions={'success':'MoveArmDown',
                                            'error':'OpenGripper'})

        smach.StateMachine.add('MoveArmDown',
                               SimpleActionState('arm_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.moveArmUpGoalCb,
                                                 result_cb=trajectory_control.generalResponseCb),
                               remapping={'joint_states':'joint_states',
                                          'distance':'move_arm_down_distance'},
                               transitions={'succeeded':'OpenGripper',
                                            'aborted':'OpenGripper',
                                            'preempted':'preempted'})

        smach.StateMachine.add('OpenGripper',
                               SimpleActionState('gripper_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.gripperControlGoalCb,
                                                 result_cb=trajectory_control.gripperControlResultCb),
                               remapping={'angle':'angle_gripper_opened',
                                          'open_gripper':'true',
                                          'close_gripper':'false'},
                               transitions={'succeeded':'DetachObject',
                                            'aborted':'place_failed',
                                            'preempted':'preempted'})

        smach.StateMachine.add('DetachObject',
                               misc_tools.AttachObjectToRobot(),
                               remapping={'collision_object':'collision_object',
                                          'attach':'false'},
                               transitions={'done':'WaitForObjectDetached'})

        smach.StateMachine.add('WaitForObjectDetached',
                               misc_tools.Wait(),
                               remapping={'duration':'wait_4sec'},
                               transitions={'done':'MoveArmIKPostPlace'})

        smach.StateMachine.add('MoveArmIKPostPlace',
                               SimpleActionState('move_arm_ik',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'post_place_pose'},
                               transitions={'succeeded':'CloseGripper',
                                            'aborted':'MoveArmDefault',
                                            'preempted':'preempted'})

        smach.StateMachine.add('CloseGripper',
                               SimpleActionState('gripper_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.gripperControlGoalCb,
                                                 result_cb=trajectory_control.gripperControlResultCb),
                               remapping={'angle':'angle_gripper_closed',
                                          'open_gripper':'false',
                                          'close_gripper':'true'},
                               transitions={'succeeded':'MoveArmDefault',
                                            'aborted':'place_failed',
                                            'preempted':'preempted'})

        smach.StateMachine.add('MoveArmDefault',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pose_arm_default'},
                               transitions={'succeeded':'placed',
                                            'aborted':'place_failed',
                                            'preempted':'preempted'})

    return sm
