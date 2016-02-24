#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('thorp_smach')
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
import tf

import geometry_msgs.msg as geometry_msgs
import manipulation_msgs.msg as manipulation_msgs
import moveit_msgs.msg as moveit_msgs
import pick_and_place_msgs.msg as pick_and_place_msgs
import sensor_msgs.msg as sensor_msgs



class Prepare(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['prepared'],
                             input_keys=['place_pose'],
                             output_keys=['place_pose'])
    def execute(self, userdata):
#        rospy.loginfo('Place pose before adaption:')
#        rospy.loginfo(userdata.place_pose)
        angle = math.atan2(userdata.place_pose.pose.position.y, userdata.place_pose.pose.position.x)
        dist = math.sqrt(math.pow(userdata.place_pose.pose.position.x, 2)
                         + math.pow(userdata.place_pose.pose.position.y, 2))
        yaw = angle
        roll = math.pi / 2
        pitch = 0.0
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        userdata.place_pose.pose.orientation = geometry_msgs.Quaternion(*quat)
#        rospy.loginfo('Place pose after adaption:')
#        rospy.loginfo(userdata.place_pose)
        return 'prepared'

@smach.cb_interface(input_keys=['object_name',
                                'place_pose'])
def placeGoalCb(userdata, goal):
    goal.group_name = "arm"
    goal.attached_object_name = userdata.object_name
    ''' place location '''
    place_loc = manipulation_msgs.PlaceLocation()
    place_loc.id = "table"
    ''' post place posture '''
    place_loc.post_place_posture.header.stamp = rospy.Time.now()
    place_loc.post_place_posture.header.frame_id = "gripper_link"
    place_loc.post_place_posture.name.append("gripper")
    place_loc.post_place_posture.position.append(1.1)
    place_loc.post_place_posture.velocity.append(0.0)
    place_loc.post_place_posture.effort.append(0.0)
    ''' place pose '''
    place_loc.place_pose = userdata.place_pose
    ''' gripper approach '''
    gripper_approach = manipulation_msgs.GripperTranslation()
    gripper_approach.direction.header.stamp = place_loc.post_place_posture.header.stamp
    gripper_approach.direction.header.frame_id = "palm_link"
    gripper_approach.direction.vector.x = 1.0
    gripper_approach.direction.vector.y = -1.0
    gripper_approach.direction.vector.z = 0.0
    gripper_approach.desired_distance = 0.10
    gripper_approach.min_distance = 0.05
    place_loc.approach = gripper_approach
    ''' gripper retreat '''
    gripper_retreat = manipulation_msgs.GripperTranslation()
    gripper_retreat.direction.header = gripper_approach.direction.header
    gripper_retreat.direction.vector.x = -1.0
    gripper_retreat.direction.vector.y = 1.0
    gripper_retreat.direction.vector.z = 0.0
    gripper_retreat.desired_distance = 0.10
    gripper_retreat.min_distance = 0.05
    place_loc.retreat = gripper_retreat
    ''' allowed touch objects '''
#    place_loc.allowed_touch_objects.append("")
    print 'place_loc'
    print place_loc
    goal.place_locations.append(place_loc)
#    goal.support_surface_name = ...
    goal.allow_gripper_support_collision = False
#    goal.path_constraints = ...
#    goal.planner_id = ...
    goal.allowed_touch_objects = []
    goal.allowed_planning_time = 10.0
#    goal.planning_options = ...
    return goal

def placeResultCb(userdata, status, result):
#    rospy.loginfo('Place status: ' + str(status))
#    rospy.loginfo('Place result:')
#    rospy.loginfo(result)
    return 'succeeded'

def createSM():
    
    sm = smach.StateMachine(outcomes=['placed',
                                      'place_failed',
                                      'preempted',
                                      'error'],
                            input_keys=['object_name',
                                        'place_pose'],
                            output_keys=['place_pose'])

    with sm:
        
        sm.userdata.pose_arm_default = geometry_msgs.PoseStamped()
        sm.userdata.pose_arm_default.header.stamp = rospy.Time.now()
        sm.userdata.pose_arm_default.header.frame_id = "base_footprint"
        sm.userdata.pose_arm_default.pose.position.x = 0.05
        sm.userdata.pose_arm_default.pose.position.y = 0.0
        sm.userdata.pose_arm_default.pose.position.z = 0.5
        sm.userdata.pose_arm_default.pose.orientation.w = 1.0
        
        smach.StateMachine.add('Prepare',
                               Prepare(),
                               remapping={'place_pose':'place_pose'},
                               transitions={'prepared':'PlaceObject'})

        smach.StateMachine.add('PlaceObject',
                               SimpleActionState('place',
                                                 moveit_msgs.PlaceAction,
                                                 goal_cb=placeGoalCb,
                                                 result_cb=placeResultCb),
                               remapping={'object_name':'object_name',
                                          'place_pose':'place_pose'},
                               transitions={'succeeded':'MoveArmToDefaultSuccess',
                                            'aborted':'MoveArmToDefaultAborted',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveArmToDefaultSuccess',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pose_arm_default'},
                               transitions={'succeeded':'placed',
                                            'aborted':'MoveArmToDefaultTryAgain',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveArmToDefaultAborted',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pose_arm_default'},
                               transitions={'succeeded':'place_failed',
                                            'aborted':'MoveArmToDefaultTryAgain',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveArmToDefaultTryAgain',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pose_arm_default'},
                               transitions={'succeeded':'error',
                                            'aborted':'error',
                                            'preempted':'preempted'})
        
    return sm
