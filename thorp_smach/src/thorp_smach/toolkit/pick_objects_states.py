import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs

from thorp_toolkit.geometry import distance_2d

from perception_states import BlockDetection
from manipulation_states import FoldArm, PickupObject, PlaceInTray

import config as cfg


def ObjectDetection():
    """  Object detection sub state machine; iterates over object_detection action state and recovery
         mechanism until an object is detected, it's preempted or there's an error (aborted outcome) """

    # TODO copy of the perception state but using BlockDetection!  actually, not in use, I think   but would solve the folding to uncover obstacles....  mmmm ...  remove if not used
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
                            output_keys=['objects', 'object_names', 'support_surf',     'blocks'])

    with sm:
        smach.StateMachine.add('CLEAR_OCTOMAP',
                               smach_ros.ServiceState('clear_octomap',
                                                      std_srvs.Empty),
                               transitions={'succeeded': 'OBJECT_DETECTION',
                                            'preempted': 'preempted',
                                            'aborted': 'OBJECT_DETECTION'})
        smach.StateMachine.add('OBJECT_DETECTION', BlockDetection(),
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


class TargetSelection(smach.State):
    """
    Select the closest object within arm reach
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['have_target', 'no_targets'],
                             input_keys=['objects', 'object_names', 'objs_to_skip'],
                             output_keys=['target'])

    def execute(self, ud):
        targets = []
        for i, obj_pose in enumerate(ud['objects'].poses):
            dist = distance_2d(obj_pose)  # assumed in arm base reference frame
            if dist <= cfg.MAX_ARM_REACH:
                targets.append((ud['object_names'][i], dist))
        targets = sorted(targets, key=lambda t: t[1])  # sort by increasing distance
        if len(targets) > ud['objs_to_skip']:
            target, dist = targets[ud['objs_to_skip']]
            rospy.loginfo("Next target will be '%s' located at %.2fm (%d skipped)", target, dist, ud['objs_to_skip'])
            ud['target'] = target
            return 'have_target'
        elif targets:
            pass  # TODO: retry if we have picked something after the last failure
        rospy.loginfo("No targets within the %.2fm arm reach (%d skipped)", cfg.MAX_ARM_REACH, ud['objs_to_skip'])
        return 'no_targets'


class SkipOneObject(smach.State):
    """
    Select the closest object within arm reach
    """

    def __init__(self, max_failures=2):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'max_failures'],
                             input_keys=['objs_to_skip'],
                             output_keys=['objs_to_skip'])
        self.max_failures = max_failures

    def execute(self, ud):
        if ud['objs_to_skip'] < self.max_failures:
            ud['objs_to_skip'] += 1
            rospy.loginfo("Skipping %d objects", ud['objs_to_skip'])
            return 'succeeded'
        rospy.logwarn("%d failures; not skipping more objects", self.max_failures)
        return 'max_failures'


def PickReachableObjs():
    """  Pick all the objects within reach and place in the tray  """

    # pick a single object sm
    pick_1_obj_sm = smach.StateMachine(outcomes=['continue', 'succeeded', 'aborted', 'preempted', 'tray_full'],
                                       input_keys=['support_surf', 'max_effort'])

    pick_1_obj_sm.userdata.objs_to_skip = 0
    with pick_1_obj_sm:
        smach.StateMachine.add('BLOCK_DETECTION', BlockDetection(),
                               transitions={'succeeded': 'SELECT_TARGET',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('SELECT_TARGET', TargetSelection(),
                               transitions={'have_target': 'PICKUP_OBJECT',
                                            'no_targets': 'succeeded'},
                               remapping={'target': 'object_name'})
        smach.StateMachine.add('PICKUP_OBJECT', PickupObject(),
                               transitions={'succeeded': 'PLACE_ON_TRAY',
                                            'preempted': 'preempted',
                                            'aborted': 'SKIP_OBJECT'})
        smach.StateMachine.add('PLACE_ON_TRAY', PlaceInTray(),
                               transitions={'succeeded': 'continue',
                                            'preempted': 'preempted',
                                            'aborted': 'CLEAR_GRIPPER',
                                            'tray_full': 'tray_full'})
        smach.StateMachine.add('CLEAR_GRIPPER', smach_ros.ServiceState('clear_gripper', std_srvs.Empty),
                               transitions={'succeeded': 'SKIP_OBJECT',
                                            'preempted': 'aborted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('SKIP_OBJECT', SkipOneObject(),
                               transitions={'succeeded': 'BLOCK_DETECTION',
                                            'max_failures': 'aborted'})

    pick_reach_objs_it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted', 'tray_full'],
                                        input_keys=[],   #'object_names'],  #, 'support_surf'],
                                        output_keys=[],
                                        it=range(25),  # kind of while true
                                        it_label='iteration',
                                        exhausted_outcome='succeeded')
    pick_reach_objs_it.userdata.max_effort = 0.3        # TODO  pick_effort in obj manip  is this really used???  should depend on the obj???
    pick_reach_objs_it.userdata.support_surf = 'table'  # TODO this comes from perception
    with pick_reach_objs_it:
        smach.Iterator.set_contained_state('', pick_1_obj_sm, loop_outcomes=['continue'])

    pick_reach_objs_sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'tray_full'],
                                            input_keys=['support_surf', 'max_effort'])
    pick_reach_objs_sm.userdata.objs_to_skip = 0
    with pick_reach_objs_sm:
        smach.StateMachine.add('PICKUP_OBJECTS', pick_reach_objs_it,
                               transitions={'succeeded': 'FOLD_ARM',
                                            'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('FOLD_ARM', FoldArm(),
                               transitions={'succeeded': 'BLOCK_DETECTION',
                                            'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BLOCK_DETECTION', BlockDetection(),
                               transitions={'succeeded': 'SELECT_TARGET',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('SELECT_TARGET', TargetSelection(),  # just used to check if there are more blocks
                               transitions={'have_target': 'PICKUP_OBJECTS',
                                            'no_targets': 'succeeded'})

    return pick_reach_objs_sm
