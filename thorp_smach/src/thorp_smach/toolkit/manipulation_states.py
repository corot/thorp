import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import control_msgs.msg as control_msgs


def FoldArm():
    """ Concurrently fold arm and close the gripper """
    sm = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                           default_outcome='succeeded',
                           outcome_map={'succeeded': {'CloseGripper': 'succeeded',
                                                      'MoveToResting': 'succeeded'},
                                        'preempted': {'CloseGripper': 'preempted',
                                                      'MoveToResting': 'preempted'},
                                        'aborted': {'CloseGripper': 'aborted',
                                                    'MoveToResting': 'aborted'}})
    with sm:
        smach.Concurrence.add('CloseGripper',
                              smach_ros.SimpleActionState('gripper_controller/gripper_action',
                                                          control_msgs.GripperCommandAction,
                                                          goal=control_msgs.GripperCommandGoal(
                                                              control_msgs.GripperCommand(0.025, 0.0))))
        smach.Concurrence.add('MoveToResting',
                              StoredConfig('resting'))

    return sm


def StoredConfig(config):
    """ Move arm into one of the stored configuration (resting, right_up, etc.) """
    return smach_ros.SimpleActionState('move_to_target',
                                       thorp_msgs.MoveToTargetAction,
                                       goal=thorp_msgs.MoveToTargetGoal(
                                           thorp_msgs.MoveToTargetGoal.NAMED_TARGET,
                                           config, None, None))


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


def PickupObject(attempts=3):
    """  Pickup a given object, retrying up to a given number of times  """
    it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                        input_keys=['object_name', 'support_surf', 'max_effort'],
                        output_keys=[],
                        it=lambda: range(0, attempts),
                        it_label='attempt',
                        exhausted_outcome='aborted')

    with it:
        sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                input_keys=['object_name', 'support_surf', 'max_effort'],
                                output_keys=[])
        with sm:
            smach.StateMachine.add('PickupObject',
                                   smach_ros.SimpleActionState('pickup_object',
                                                               thorp_msgs.PickupObjectAction,
                                                               goal_slots=['object_name', 'support_surf', 'max_effort'],
                                                               result_slots=[]),
                                   remapping={'object_name': 'object_name',
                                              'support_surf': 'support_surf',
                                              'max_effort': 'max_effort'},
                                   transitions={'succeeded': 'succeeded',
                                                'preempted': 'preempted',
                                                'aborted': 'ClearOctomap'})

            smach.StateMachine.add('ClearOctomap',
                                   smach_ros.ServiceState('clear_octomap',
                                                          std_srvs.Empty),
                                   transitions={'succeeded': 'continue',
                                                'preempted': 'preempted',
                                                'aborted': 'aborted'})

        # TODOs:
        #  - we should open the gripper, in case we have picked an object
        #  - check error and, if collision between parts of the arm, move a bit the arm  -->  not enough info
        #  - this doesn't make too much sense as a loop... better try all our tricks and exit
        #  - can I reuse the same for place and MoveToTarget???

        smach.Iterator.set_contained_state('', sm, loop_outcomes=['continue'])

    return it


def PlaceObject(attempts=3):
    """  Place a given object, retrying up to a given number of times  """
    it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                        input_keys=['object_name', 'support_surf', 'place_pose'],
                        output_keys=[],
                        it=lambda: range(0, attempts),
                        it_label='attempt',
                        exhausted_outcome='aborted')

    with it:
        sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                input_keys=['object_name', 'support_surf', 'place_pose'],
                                output_keys=[])
        with sm:
            smach.StateMachine.add('PlaceObject',
                                   smach_ros.SimpleActionState('place_object',
                                                               thorp_msgs.PlaceObjectAction,
                                                               goal_slots=['object_name', 'support_surf', 'place_pose'],
                                                               result_slots=[]),
                                   remapping={'object_name': 'object_name',
                                              'support_surf': 'support_surf',
                                              'place_pose': 'place_pose'},
                                   transitions={'succeeded': 'succeeded',
                                                'preempted': 'preempted',
                                                'aborted': 'ClearOctomap'})

            smach.StateMachine.add('ClearOctomap',
                                   smach_ros.ServiceState('clear_octomap',
                                                          std_srvs.Empty),
                                   transitions={'succeeded': 'continue',
                                                'preempted': 'preempted',
                                                'aborted': 'aborted'})

        # TODOs:
        #  - we should open the gripper, in case we have picked an object
        #  - check error and, if collision between parts of the arm, move a bit the arm  -->  not enough info
        #  - this doesn't make too much sense as a loop... better try all our tricks and exit
        #  - can I reuse the same for place and MoveToTarget???

        smach.Iterator.set_contained_state('', sm, loop_outcomes=['continue'])

    return it
