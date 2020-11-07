import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs
import geometry_msgs.msg as geometry_msgs
from turtlebot_arm_block_manipulation.msg import BlockDetectionAction

from copy import deepcopy
from collections import namedtuple

from thorp_toolkit.geometry import TF2, to_transform, transform_pose, apply_transform, create_2d_pose, distance_2d
from thorp_toolkit.visualization import Visualization

from perception_states import MonitorTables
from navigation_states import GetRobotPose, GoToPose, ExePath, PoseAsPath
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


class CalcPickPoses(smach.State):
    """
    Calculate all picking locations around a table at a given distance.
    In current simple version, just the in front of each of the four sides.
    """

    def __init__(self, distance):
        smach.State.__init__(self, outcomes=['succeeded', 'no_valid_table'],
                             input_keys=['detected_table', 'detected_table_pose', 'robot_pose'],
                             output_keys=['picking_poses', 'closest_picking_pose'])
        self.distance = distance
        self.poses_viz = rospy.Publisher('manipulation/picking_poses', geometry_msgs.PoseArray, queue_size=1)

    def execute(self, ud):
        table = ud['detected_table']
        table_pose = ud['detected_table_pose']  # expected on map frame, as robot_pose
        if not table or not table_pose:
            rospy.logerr("Detected table contains None!")
            return 'no_table'
        p_x = self.distance + table.depth / 2.0
        n_x = - p_x
        p_y = self.distance + table.width / 2.0
        n_y = - p_y
        table_tf = to_transform(table_pose, 'table_frame')
        TF2().publish_transform(table_tf)
        table_tf.transform.translation.z = 0.0
        ud['picking_poses'] = {}
        closest_pose = None
        closest_dist = float('inf')
        from math import pi
        poses = [('p_x', create_2d_pose(p_x, 0.0, -pi, 'table_frame')),
                 ('n_x', create_2d_pose(n_x, 0.0, 0.0, 'table_frame')),
                 ('p_y', create_2d_pose(0.0, p_y, -pi/2.0, 'table_frame')),
                 ('n_y', create_2d_pose(0.0, n_y, +pi/2.0, 'table_frame'))]
        pose_array = geometry_msgs.PoseArray()  # for visualization
        picking_poses = {}
        for name, pose in poses:
            pose = transform_pose(pose, table_tf)
            pose_array.poses.append(pose.pose)
            picking_poses[name] = pose
            dist = distance_2d(pose, ud['robot_pose'])
            if dist < closest_dist:
                closest_pose = pose
                closest_dist = dist
        ud['picking_poses'] = picking_poses
        ud['closest_picking_pose'] = closest_pose
        pose_array.header = deepcopy(closest_pose.header)
        pose_array.poses.append(deepcopy(closest_pose.pose))
        pose_array.poses[-1].position.z += 0.025  # mark the closest pose
        self.poses_viz.publish(pose_array)
        return 'succeeded'


# Object to pick and pick location named tuples; required for grouping objects  TODO make local if I don't export as output key
Object = namedtuple('Object', ['dist', 'name', 'pose'])
PickLoc = namedtuple('PickLoc', ['size', 'name', 'pose', 'arm_pose', 'objs'])


class GroupObjects(smach.State):
    """
    Group detected objects reachable from picking location (within arm's reach).
    We first eliminate the locations without objects only reachable from there.
    If an object can be reached from two of the remaining locations, we choose
    the one that places the object closer to the robot arm.
    """
    def __init__(self, manip_frame):
        smach.State.__init__(self, outcomes=['succeeded', 'no_objects'],
                             input_keys=['objects', 'table', 'picking_poses'],
                             output_keys=['picking_plan'])
        self.manip_frame = manip_frame

    def execute(self, ud):
        bfp_to_arm_tf = TF2().lookup_transform(self.manip_frame, 'base_footprint')
        arm_to_map_tf = TF2().lookup_transform(self.manip_frame, 'map')
        pick_locs = []
        for name, pose in ud['picking_poses'].items():
            arm_pose_mrf = apply_transform(pose, bfp_to_arm_tf)
            # detected objects poses are in arm reference, so their modulo is the distance to the arm
            objs = []
            for i, obj_pose in enumerate(ud['objects'].poses):
                obj_pose_mrf = transform_pose(obj_pose, arm_to_map_tf)
                dist = distance_2d(obj_pose_mrf, arm_pose_mrf)  # both on map rf
                if dist <= cfg.MAX_ARM_REACH:
                    objs.append(Object(dist, 'block' + str(i + 1), obj_pose))
            objs = sorted(objs, key=lambda o: o.dist)  # sort by increasing distance from the arm
            pick_locs.append(PickLoc(len(objs), name, pose, arm_pose_mrf, objs))
        sorted_pls = sorted(pick_locs, key=lambda pl: pl.size)  # sort by increasing number of reachable objects
        self.viz_pick_locs(sorted_pls)

        # traverse pick locations from less to more populated, and discard those containing only duplicated objects
        # (or no objects at all), so we can skip them
        pls_to_remove = []
        for ploc in sorted_pls:
            dup_objs = 0
            for obj in ploc.objs:
                for other_pl in [pl for pl in sorted_pls if pl != ploc and pl not in pls_to_remove]:
                    if obj.name in [o.name for o in other_pl.objs]:
                        dup_objs += 1
            if dup_objs == ploc.size:
                pls_to_remove.append(ploc)
        for ploc in pls_to_remove:
            sorted_pls.remove(ploc)
        self.viz_pick_locs(sorted_pls)

        # remove duplicated objects from the location where they are at the longest distance to the arm
        objs_to_remove = []
        for ploc in sorted_pls:
            dup_objs = 0
            for obj in ploc.objs:
                for other_pl in [pl for pl in sorted_pls if pl != ploc]:
                    for other_obj in other_pl.objs:
                        if obj.name == other_obj.name:
                            if obj.dist >= other_obj.dist:
                                obj_to_rm = (ploc, obj)
                            else:
                                obj_to_rm = (other_pl, other_obj)
                            if obj_to_rm not in objs_to_remove:
                                objs_to_remove.append(obj_to_rm)
                            dup_objs += 1
        for ploc, obj in objs_to_remove:
            ploc.objs.remove(obj)
        # update pick locations objects count (a bit cumbersome with named tuples)
        for i, ploc in enumerate(sorted_pls):
            if ploc.size != len(ploc.objs):
                sorted_pls[i] = ploc._replace(size=len(ploc.objs))
        self.viz_pick_locs(sorted_pls)
        ud['picking_plan'] = sorted_pls
        return 'succeeded'

    def viz_pick_locs(self, pick_locs):
        for pl in pick_locs:
            color = Visualization.rand_color(0.4)
            Visualization().add_disc_marker(pl.arm_pose, [cfg.MAX_ARM_REACH * 2.0] * 2, color)

            text_pose = deepcopy(pl.arm_pose)
            text_pose.pose.position.z += 0.15
            Visualization().add_text_marker(text_pose, pl.name + ' ' + str(pl.size), 0.2, color)

            for obj in pl.objs:
                text_pose = geometry_msgs.PoseStamped()
                text_pose.header.frame_id = self.manip_frame
                text_pose.pose = deepcopy(obj.pose)
                text_pose.pose.position.z += 0.05
                Visualization().add_text_marker(text_pose, pl.name + ' ' + obj.name, 0.1, color)

        Visualization().publish_markers()


def GatherObjects():
    """
    Object gatherer SM:
     - explore house while looking for tables
     - approach each detected table and pick all the objects of the chosen shape
    """

    # explore a single room
    approach_table_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                       connector_outcome='succeeded',
                                       input_keys=['detected_table', 'detected_table_pose'],
                                       output_keys=['picking_poses', 'closest_picking_pose'])
    with approach_table_sm:
        smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
        smach.Sequence.add('CALC_PICK_POSES', CalcPickPoses(cfg.PICKING_DIST_TO_TABLE),
                           transitions={'no_valid_table': 'aborted'})
        smach.Sequence.add('APPROACH_TABLE', GoToPose(dist_tolerance=cfg.LOOSE_DIST_TOLERANCE,     # just approach
                                                      angle_tolerance=cfg.LOOSE_ANGLE_TOLERANCE),  # the table
                           transitions={'aborted': 'aborted',
                                        'preempted': 'preempted'},
                           remapping={'target_pose': 'closest_picking_pose'})
        smach.Sequence.add('DETECT_TABLES', MonitorTables(),
                           transitions={'aborted': 'aborted'})
        smach.Sequence.add('POSES_AS_PATH', PoseAsPath(),
                           remapping={'pose': 'closest_picking_pose'})
        smach.Sequence.add('ALIGN_TO_TABLE', ExePath(dist_tolerance=cfg.TIGHT_DIST_TOLERANCE,     # we try to
                                                     angle_tolerance=cfg.TIGHT_ANGLE_TOLERANCE),  # be precise
                           transitions={'aborted': 'aborted',
                                        'preempted': 'preempted'})

    # explore a single room
    pick_objects_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                     connector_outcome='succeeded',
                                     input_keys=['detected_table', 'picking_poses'])
    pick_objects_sm.userdata.frame = rospy.get_param('~arm_link', 'arm_base_link')
    pick_objects_sm.userdata.table_height = rospy.get_param('~table_height', -0.03)
    pick_objects_sm.userdata.block_size = rospy.get_param('~block_size', 0.025)

    with pick_objects_sm:  # TODO iterate until there are no objects left
        smach.Sequence.add('DETECT_OBJECTS',  # TODO: will replace with a proper object detection once I have it
                           smach_ros.SimpleActionState('block_detection',
                                                       BlockDetectionAction,
                                                       goal_slots=['frame', 'table_height', 'block_size'],
                                                       result_slots=['blocks']),
                           transitions={'aborted': 'aborted',
                                        'preempted': 'preempted'},
                           remapping={'blocks': 'detected_objects'})
        smach.Sequence.add('GROUP_OBJECTS', GroupObjects(pick_objects_sm.userdata.frame),
                           transitions={'no_objects': 'aborted'},
                           remapping={'objects': 'detected_objects',
                                      'table': 'detected_table'})

    # Full SM: plan rooms visit sequence and explore each room in turn
    sm = smach.StateMachine(outcomes=['succeeded',
                                      'aborted',
                                      'preempted'],
                            input_keys=['detected_table', 'detected_table_pose'])
    with sm:
        smach.StateMachine.add('APPROACH_TABLE', approach_table_sm,
                               transitions={'succeeded': 'PICK_OBJECTS',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('PICK_OBJECTS', pick_objects_sm,
                               transitions={'succeeded': 'succeeded',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
    return sm
