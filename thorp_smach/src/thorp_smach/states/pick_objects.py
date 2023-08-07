from random import uniform

import rospy
import smach

from thorp_toolkit.geometry import TF2, distance_2d

from .perception import ObjectDetection
from .manipulation import ClearGripper, ClearPlanningScene, FoldArm, PickupObject, PlaceOnTray

from ..containers.do_on_exit import DoOnExit as DoOnExitContainer
from .. import config as cfg


class TargetSelection(smach.State):
    """
    Select the closest object within arm reach
    'failures' dictionary lists all previous pick failures for each object. If there aren't non-attempted
    targets at hand, we will retry the object with fewer failures. If all have exhausted the max number of
    retries, we return 'no_targets'.
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['have_target', 'no_targets'],
                             input_keys=['objects', 'failures'],
                             output_keys=['target', 'tightening'])
        manip_frame = rospy.get_param('~planning_frame', 'arm_base_link')
        self.arm_on_bfp_rf = TF2().transform_pose(None, manip_frame, 'base_footprint')

    def execute(self, ud):
        targets = []
        for obj in ud['objects']:
            dist = distance_2d(obj.pose, self.arm_on_bfp_rf)  # assumed in arm base reference frame
            if dist <= (cfg.MAX_ARM_REACH - 3e-3):  # 3 mm safety margin to account for perception noise
                targets.append((obj, dist))
        targets = sorted(targets, key=lambda t: t[1])  # sort by increasing distance
        for target, dist in targets:
            if target.id in ud['failures'].keys():
                continue
            rospy.loginfo("Next target: '%s', located at %.2f m from the arm", target.id, dist)
            ud['target'] = target
            ud['tightening'] = cfg.GRIPPER_TIGHTENING
            return 'have_target'

        rospy.loginfo("No new targets within the %.2f m arm reach", cfg.MAX_ARM_REACH)
        # all targets have already failed at least once; try with the one with less previous failures
        targets = [(t, d, ud['failures'][t.id]) for t, d in targets]
        targets = sorted(targets, key=lambda t: t[2])  # sort by increasing number of failed picks
        for target, dist, failures in targets:
            # we add a random extra tightening, the bigger the more we retry, but with some randomness
            if failures < cfg.PICKING_MAX_FAILURES:
                extra_tightening = uniform(-cfg.GRIPPER_TIGHTENING, cfg.GRIPPER_TIGHTENING * failures)
                rospy.loginfo("Retrying target '%s' (%d previous failures; %.1f mm of extra tightening)",
                              target.id, failures, extra_tightening * 1000)
                ud['target'] = target
                ud['tightening'] = cfg.GRIPPER_TIGHTENING + extra_tightening
                return 'have_target'
        rospy.loginfo("No targets to retry (failed less than %d times)", cfg.PICKING_MAX_FAILURES)
        return 'no_targets'


class RecordFailure(smach.State):
    """
    Increase by one the number of picking failures for a given object
    Return always 'succeeded'
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['failures', 'object'],
                             output_keys=['failures'])

    def execute(self, ud):
        if ud['object'].id not in ud['failures'].keys():
            ud['failures'][ud['object'].id] = 1
        else:
            ud['failures'][ud['object'].id] += 1
        failures = ud['failures'][ud['object'].id]
        assert failures <= cfg.PICKING_MAX_FAILURES, "We are retrying more than max failures"
        if failures < cfg.PICKING_MAX_FAILURES:
            rospy.loginfo("Pick object '%s' failed %d time(s)", ud['object'].id, failures)
        else:
            rospy.logwarn("Pick object '%s' failed %d times; giving up", ud['object'].id, failures)
        return 'succeeded'


class ClearFailures(smach.State):
    """
    Clear the picking failures for a given object (normally after successfully picking it)
    Always returns 'succeeded', even if the object was not listed on 'failures' dictionary.
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['object', 'failures'],
                             output_keys=['failures'])

    def execute(self, ud):
        try:
            del ud['failures'][ud['object'].id]
            rospy.loginfo("Object '%s' cleared from failures dictionary", ud['object'].id)
        except KeyError:
            rospy.logwarn("Object '%s' not listed on failures dictionary", ud['object'].id)
        return 'succeeded'


class PickReachableObjs(DoOnExitContainer):
    """  Pick all the objects within reach and place in the tray  """

    def __init__(self):
        super(PickReachableObjs, self).__init__(outcomes=['succeeded', 'aborted', 'preempted', 'tray_full'],
                                                input_keys=['object_types'],
                                                output_keys=['failures'])

        # pick a single object sm
        pick_1_obj_sm = smach.StateMachine(outcomes=['continue', 'succeeded', 'aborted', 'preempted', 'tray_full'],
                                           input_keys=['object_types', 'failures'],
                                           output_keys=['failures'])

        pick_1_obj_sm.userdata.max_effort = cfg.GRIPPER_MAX_EFFORT
        with pick_1_obj_sm:
            smach.StateMachine.add('DETECT_OBJECTS', ObjectDetection(),
                                   transitions={'succeeded': 'SELECT_TARGET',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})
            smach.StateMachine.add('SELECT_TARGET', TargetSelection(),
                                   transitions={'have_target': 'PICKUP_OBJECT',
                                                'no_targets': 'succeeded'},
                                   remapping={'target': 'object'})
            smach.StateMachine.add('PICKUP_OBJECT', PickupObject(),
                                   transitions={'succeeded': 'PLACE_ON_TRAY',
                                                'preempted': 'preempted',
                                                'aborted': 'RECORD_FAILURE'})
            smach.StateMachine.add('PLACE_ON_TRAY', PlaceOnTray(),
                                   transitions={'succeeded': 'CLEAR_FAILURES',
                                                'preempted': 'preempted',
                                                'aborted': 'CLEAR_GRIPPER',
                                                'tray_full': 'tray_full'})
            smach.StateMachine.add('CLEAR_GRIPPER', ClearGripper(),
                                   transitions={'succeeded': 'RECORD_FAILURE',
                                                'preempted': 'aborted',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('RECORD_FAILURE', RecordFailure(),
                                   transitions={'succeeded': 'continue'})
            smach.StateMachine.add('CLEAR_FAILURES', ClearFailures(),
                                   transitions={'succeeded': 'continue'})

        pick_reach_objs_it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted', 'tray_full'],
                                            input_keys=['object_types', 'failures'],
                                            output_keys=['failures'],
                                            it=range(25),  # kind of while true
                                            it_label='iteration',
                                            exhausted_outcome='succeeded')
        with pick_reach_objs_it:
            smach.Iterator.set_contained_state('', pick_1_obj_sm, loop_outcomes=['continue'])

        with self:
            smach.StateMachine.add('PICKUP_OBJECTS', pick_reach_objs_it,
                                   transitions={'succeeded': 'FOLD_ARM',
                                                'preempted': 'preempted',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('FOLD_ARM', FoldArm(),
                                   transitions={'succeeded': 'DETECT_OBJECTS',
                                                'preempted': 'preempted',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('DETECT_OBJECTS', ObjectDetection(),
                                   transitions={'succeeded': 'SELECT_TARGET',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})
            smach.StateMachine.add('SELECT_TARGET', TargetSelection(),  # just used to check if there are objects left
                                   transitions={'have_target': 'PICKUP_OBJECTS',  # restart picking if so
                                                'no_targets': 'succeeded'})
            DoOnExitContainer.add_finally('CLEAR_P_SCENE', ClearPlanningScene())
            DoOnExitContainer.add_finally('FINAL_FOLD_ARM', FoldArm())

    def execute(self, parent_ud=smach.UserData()):
        self.userdata.failures = {}  # keep track of pick failures for each object
        return super(PickReachableObjs, self).execute(parent_ud)
