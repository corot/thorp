import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import control_msgs.msg as control_msgs
import geometry_msgs.msg as geometry_msgs

from copy import deepcopy
from collections import namedtuple
from visualization_msgs.msg import Marker, MarkerArray

from thorp_toolkit.geometry import TF2, to_transform, transform_pose, apply_transform,\
                                   create_2d_pose, create_3d_pose, distance_2d
from thorp_toolkit.visualization import Visualization

import config as cfg


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
        smach.StateMachine.add('ClearOctomap',
                               smach_ros.ServiceState('clear_octomap',
                                                      std_srvs.Empty),
                               transitions={'succeeded': 'ObjectDetection',
                                            'preempted': 'preempted',
                                            'aborted': 'ObjectDetection'})

        smach.StateMachine.add('ObjectDetection',
                               smach_ros.SimpleActionState('object_detection',
                                                           thorp_msgs.DetectObjectsAction,
                                                           goal_slots=['output_frame'],
                                                           result_slots=['objects', 'object_names', 'support_surf']),
                               remapping={'output_frame': 'output_frame',
                                          'object_names': 'object_names',
                                          'support_surf': 'support_surf'},
                               transitions={'succeeded': 'ObjDetectedCondition',
                                            'preempted': 'preempted',
                                            'aborted': 'aborted'})

        smach.StateMachine.add('ObjDetectedCondition',
                               ObjDetectedCondition(),
                               remapping={'object_names': 'object_names'},
                               transitions={'satisfied': 'succeeded',
                                            'preempted': 'preempted',
                                            'fold_arm': 'FoldArm',
                                            'retry': 'ClearOctomap'})

        smach.StateMachine.add('FoldArm',
                               FoldArm(),
                               transitions={'succeeded': 'ClearOctomap',
                                            'preempted': 'preempted',
                                            'aborted': 'ClearOctomap'})

    return sm
