#!/usr/bin/env python

from state_machines_imports import *
from thorp_smach.pick_and_place_tools import trajectory_control, misc_tools
import trajectory_msgs, control_msgs, std_srvs
import std_srvs
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class PrepareGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['prepared'],
                             input_keys=['pan_angle',
                                         'tilt_angle'],
                             output_keys=['head_goal_trajectory'])
    def execute(self, userdata):
        head_goal_trajectory = JointTrajectory()
        head_goal_trajectory.header.frame_id = "move_head"
        head_goal_trajectory.header.stamp = rospy.Time.now()
        head_goal_trajectory.joint_names.append("head_pan")
        head_goal_trajectory.joint_names.append("head_tilt")
        waypoint = JointTrajectoryPoint()
        waypoint.positions.append(userdata.pan_angle)
        waypoint.positions.append(userdata.tilt_angle)
        waypoint.velocities.append(2.0)
        waypoint.velocities.append(2.0)
        waypoint.accelerations.append(0.0)
        waypoint.accelerations.append(0.0)
        head_goal_trajectory.points.append(waypoint)
        userdata.head_goal_trajectory = head_goal_trajectory
        return 'prepared'


def createSM():
    sm_take_coll_map_snapshot = smach.StateMachine(outcomes=['done',
                                                             'preempted',
                                                             'aborted'],
                                                   input_keys=['pan_angle',
                                                               'tilt_angle'],
                                                   output_keys=['error_message',
                                                                'error_code'])
    with sm_take_coll_map_snapshot:
        sm_take_coll_map_snapshot.userdata.head_goal_trajectory = JointTrajectory()
        # wait
        sm_take_coll_map_snapshot.userdata.wait_0sec = 0.0
        sm_take_coll_map_snapshot.userdata.wait_1sec = 1.0
        sm_take_coll_map_snapshot.userdata.wait_5sec = 5.0
        sm_take_coll_map_snapshot.userdata.wait_6sec = 6.0
        
        smach.StateMachine.add('PrepareHeadGoal',
                               PrepareGoal(),
                               remapping={'pan_angle':'pan_angle',
                                          'tilt_angle':'tilt_angle',
                                          'head_goal_trajectory':'head_goal_trajectory'},
                               transitions={'prepared':'MoveHead'})
        smach.StateMachine.add('MoveHead',
                               SimpleActionState('/korus/head_controller',
                                                 control_msgs.msg.FollowJointTrajectoryAction,
                                                 goal_slots=['trajectory'],
                                                 result_cb=trajectory_control.generalResponseCb,
                                                 output_keys=['error_code']),
                               remapping={'trajectory':'head_goal_trajectory',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveHeadErrorCode',
                                            'aborted':'ParseMoveHeadErrorCode',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('ParseMoveHeadErrorCode',
                               trajectory_control.FollowJointTrajectoryErrorCodesParser(),
                               transitions={'success':'WaitForPointcloudAlignment',
                                            'parsed':'aborted'},
                               remapping={'error_code':'error_code',
                                          'error_message':'error_message'})
        
        smach.StateMachine.add('WaitForPointcloudAlignment', misc_tools.Wait(),
                               remapping={'duration':'wait_0sec'},
                               transitions={'done':'TakeSnapshot'})
        
        smach.StateMachine.add('TakeSnapshot',
                               ServiceState('/korus/pointcloud_throttle_service/pass_on_pointcloud',
                                            std_srvs.srv.Empty()),
                                            transitions={'succeeded':'done',
                                                         'preempted':'preempted',
                                                         'aborted':'aborted'})
    return sm_take_coll_map_snapshot