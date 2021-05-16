from math import ceil
from random import randrange, uniform

import rospy
import smach

from thorp_toolkit.geometry import TF2, distance_2d

from perception import ObjectDetection
from manipulation import ClearGripper, ClearPlanningScene, FoldArm, PickupObject, PlaceInTray

from thorp_smach import config as cfg


class TargetSelection(smach.State):
    """
    Select the closest object within arm reach
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['have_target', 'no_targets'],
                             input_keys=['objects', 'objs_to_skip'],
                             output_keys=['target', 'tightening'])
        manip_frame = rospy.get_param('~rec_objects_frame', 'arm_base_link')
        self.arm_on_bfp_rf = TF2().transform_pose(None, manip_frame, 'base_footprint')
        self.retries = 0

    def execute(self, ud):
        targets = []
        for i, obj in enumerate(ud['objects']):
            obj_pose = obj.primitive_poses[0]
            dist = distance_2d(obj_pose, self.arm_on_bfp_rf)  # assumed in arm base reference frame
            if dist <= (cfg.MAX_ARM_REACH - 3e-3):  # 3 mm safety margin to account for perception noise
                targets.append((obj, dist))
        targets = sorted(targets, key=lambda t: t[1])  # sort by increasing distance
        if len(targets) > ud['objs_to_skip']:
            self.retries = 0
            target, dist = targets[ud['objs_to_skip']]
            rospy.loginfo("Next target: '%s', located at %.2fm (%d skipped)", target.id, dist, ud['objs_to_skip'])
            ud['target'] = target
            ud['tightening'] = cfg.GRIPPER_TIGHTENING
            return 'have_target'
        elif targets:
            # retry failed objects at random order; add a random extra tightening, the bigger the more we retry
            self.retries += 1
            target, dist = targets[randrange(0, min(len(targets), ud['objs_to_skip']))]
            rospy.loginfo("Retrying target '%s', located at %.2fm (%d to retry)", target.id, dist, ud['objs_to_skip'])
            ud['target'] = target
            # ud['tightening'] = cfg.GRIPPER_TIGHTENING + uniform(-cfg.GRIPPER_TIGHTENING, cfg.GRIPPER_TIGHTENING * self.retries)
            # tightening_factor = ceil(self.retries / float(ud['objs_to_skip']))
            extra_tightening = uniform(-cfg.GRIPPER_TIGHTENING, cfg.GRIPPER_TIGHTENING * self.retries)
            print self.retries, len(targets), ud['objs_to_skip'], extra_tightening
            ud['tightening'] = cfg.GRIPPER_TIGHTENING + extra_tightening
            rospy.loginfo("%d retries so far; %.1f mm of extra tightening", self.retries, extra_tightening * 1000)
            return 'have_target'
        rospy.loginfo("No targets within the %.2fm arm reach (%d skipped)", cfg.MAX_ARM_REACH, ud['objs_to_skip'])
        return 'no_targets'


class SkipOneObject(smach.State):
    """
    Increase by one the number of objects to skip (normally after a failed picking)
    Return 'succeeded' if we can keep trying, or 'max_failures' if we exhausted the allowed picking failures
    """

    def __init__(self, max_failures=cfg.PICKING_MAX_FAILURES):
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


class PickReachableObjs(smach.StateMachine):
    """  Pick all the objects within reach and place in the tray  """

    def __init__(self):
        super(PickReachableObjs, self).__init__(outcomes=['succeeded', 'aborted', 'preempted', 'tray_full'],
                                                input_keys=['object_types'])

        # pick a single object sm
        pick_1_obj_sm = smach.StateMachine(outcomes=['continue', 'succeeded', 'aborted', 'preempted', 'tray_full'],
                                           input_keys=['object_types', 'objs_to_skip'])

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
                                                'aborted': 'SKIP_OBJECT'})
            smach.StateMachine.add('PLACE_ON_TRAY', PlaceInTray(),
                                   transitions={'succeeded': 'continue',
                                                'preempted': 'preempted',
                                                'aborted': 'CLEAR_GRIPPER',
                                                'tray_full': 'tray_full'})
            smach.StateMachine.add('CLEAR_GRIPPER', ClearGripper(),
                                   transitions={'succeeded': 'SKIP_OBJECT',
                                                'preempted': 'aborted',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('SKIP_OBJECT', SkipOneObject(),
                                   transitions={'succeeded': 'DETECT_OBJECTS',
                                                'max_failures': 'aborted'})

        pick_reach_objs_it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted', 'tray_full'],
                                            input_keys=['object_types', 'objs_to_skip'],
                                            output_keys=[],
                                            it=range(25),  # kind of while true
                                            it_label='iteration',
                                            exhausted_outcome='succeeded')
        with pick_reach_objs_it:
            smach.Iterator.set_contained_state('', pick_1_obj_sm, loop_outcomes=['continue'])

        self.userdata.objs_to_skip = 0
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
            smach.StateMachine.add('SELECT_TARGET', TargetSelection(),  # just used to check if there are more objects
                                   transitions={'have_target': 'PICKUP_OBJECTS',
                                                'no_targets': 'CLEAR_P_SCENE'})
            smach.StateMachine.add('CLEAR_P_SCENE', ClearPlanningScene())
