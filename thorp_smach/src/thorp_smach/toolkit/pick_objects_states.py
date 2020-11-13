import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs

from turtlebot_arm_block_manipulation.msg import BlockDetectionAction

from thorp_toolkit.geometry import distance_2d

from manipulation_states import FoldArm, PickupObject, PlaceInTray

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
                            output_keys=['objects', 'object_names', 'support_surf',     'blocks'])

    with sm:
        smach.StateMachine.add('CLEAR_OCTOMAP',
                               smach_ros.ServiceState('clear_octomap',
                                                      std_srvs.Empty),
                               transitions={'succeeded': 'OBJECT_DETECTION',
                                            'preempted': 'preempted',
                                            'aborted': 'OBJECT_DETECTION'})

        # app config
        sm.userdata.frame = rospy.get_param('~arm_link', 'arm_base_link')
        sm.userdata.table_height = rospy.get_param('~table_height', -0.03)
        sm.userdata.block_size = rospy.get_param('~block_size', 0.025)

        def result_cb(ud, status, result):
            #ud['blocks'] = result.blocks
            ud['objects'] = result.blocks
            ud['object_names'] = ['block' + str(i) for i in range(1, len(result.blocks.poses) + 1)]
            ud['support_surf'] = 'table'

        smach.StateMachine.add('OBJECT_DETECTION',
                               smach_ros.SimpleActionState('block_detection',
                                                           BlockDetectionAction,
                                                           goal_slots=['frame', 'table_height', 'block_size'],
                                                           result_slots=['blocks'],
                                                           result_cb=result_cb,
                                                           output_keys=['objects', 'object_names', 'support_surf']),
                               transitions={'succeeded': 'OBJ_DETECTED_COND',
                                            'preempted': 'preempted',
                                            'aborted': 'aborted'})
        #   TODO cambiazo,,,,  borrar esto cuando tenga decente obj rec
        #
        # smach.StateMachine.add('OBJECT_DETECTION',
        #                        smach_ros.SimpleActionState('object_detection',
        #                                                    thorp_msgs.DetectObjectsAction,
        #                                                    goal_slots=['output_frame'],
        #                                                    result_slots=['objects', 'object_names', 'support_surf']),
        #                        remapping={'output_frame': 'output_frame',
        #                                   'object_names': 'object_names',
        #                                   'support_surf': 'support_surf'},
        #                        transitions={'succeeded': 'OBJ_DETECTED_COND',
        #                                     'preempted': 'preempted',
        #                                     'aborted': 'aborted'})

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

    def result_cb(ud, status, result):
        ud['blocks'] = result.blocks
        ud['objects'] = result.blocks
        ud['object_names'] = ['block' + str(i) for i in range(1, len(result.blocks.poses) + 1)]

    # pick a single object sm
    pick_1_obj_sm = smach.StateMachine(outcomes=['continue', 'succeeded', 'aborted', 'preempted', 'tray_full'],
                                       input_keys=['support_surf', 'max_effort'])
    pick_1_obj_sm.userdata.frame = rospy.get_param('~arm_link', 'arm_base_link')
    pick_1_obj_sm.userdata.table_height = rospy.get_param('~table_height', -0.03)
    pick_1_obj_sm.userdata.block_size = rospy.get_param('~block_size', 0.025)
    pick_1_obj_sm.userdata.objs_to_skip = 0
    with pick_1_obj_sm:
        smach.StateMachine.add('BLOCK_DETECTION',
                               smach_ros.SimpleActionState('block_detection',
                                                           BlockDetectionAction,
                                                           goal_slots=['frame', 'table_height', 'block_size'],
                                                           result_slots=['blocks'],
                                                           result_cb=result_cb,
                                                           output_keys=['blocks', 'objects', 'object_names']),
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

    it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted', 'tray_full'],
                        input_keys=[],   #'object_names'],  #, 'support_surf'],
                        output_keys=[],
                        it=range(25),  # kind of while true
                        it_label='iteration',
                        exhausted_outcome='succeeded')
    it.userdata.max_effort = 0.3        # TODO  pick_effort in obj manip  is this really used???  should depend on the obj???
    it.userdata.support_surf = 'table'  # TODO this comes from perception
    with it:
        smach.Iterator.set_contained_state('', pick_1_obj_sm, loop_outcomes=['continue'])
    return it
