import rospy
import smach

import geometry_msgs.msg as geometry_msgs

from copy import deepcopy
from itertools import permutations
from collections import namedtuple

from thorp_toolkit.geometry import TF2, to_transform, translate_pose, transform_pose, create_2d_pose, distance_2d
from thorp_toolkit.transform import Transform
from thorp_toolkit.visualization import Visualization

from common import SetNamedConfig, DismissNamedConfig
from costmaps import TableAsObstacle
from perception import MonitorTables, ObjectDetection, ObjectsDetected
from navigation import GetRobotPose, GoToPose, AlignToTable, DetachFromTable
from pick_objects import PickReachableObjs
from thorp_smach.containers.do_on_exit import DoOnExit as DoOnExitContainer

from thorp_smach import config as cfg


class TableSidesPoses(smach.State):
    """
    Calculate the four locations around a squared table at a given distance.
    """

    def __init__(self, distance):
        smach.State.__init__(self, outcomes=['succeeded', 'no_valid_table'],
                             input_keys=['table', 'table_pose', 'robot_pose'],
                             output_keys=['poses', 'closest_pose'])
        self.distance = distance
        self.poses_viz = rospy.Publisher('manipulation/picking_poses', geometry_msgs.PoseArray, queue_size=1)

    def execute(self, ud):
        table = ud['table']
        table_pose = ud['table_pose']  # expected on map frame, as robot_pose
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
        closest_pose = None
        closest_dist = float('inf')
        from math import pi
        poses = [('p_x', create_2d_pose(p_x, 0.0, -pi, 'table_frame')),
                 ('n_x', create_2d_pose(n_x, 0.0, 0.0, 'table_frame')),
                 ('p_y', create_2d_pose(0.0, p_y, -pi/2.0, 'table_frame')),
                 ('n_y', create_2d_pose(0.0, n_y, +pi/2.0, 'table_frame'))]
        pose_array = geometry_msgs.PoseArray()  # for visualization
        sides_poses = {}
        for name, pose in poses:
            pose = transform_pose(pose, table_tf)
            pose_array.poses.append(deepcopy(pose.pose))
            pose_array.poses[-1].position.z += 0.025  # raise over costmap to make it visible
            sides_poses[name] = pose
            dist = distance_2d(pose, ud['robot_pose'])
            if dist < closest_dist:
                closest_pose = pose
                closest_dist = dist
        ud['poses'] = sides_poses
        ud['closest_pose'] = closest_pose
        pose_array.header = deepcopy(closest_pose.header)
        pose_array.poses.append(deepcopy(closest_pose.pose))
        pose_array.poses[-1].position.z += 0.05  # remark the closest pose with a double arrow
        self.poses_viz.publish(pose_array)
        return 'succeeded'


# Object to pick and pick location named tuples; required for grouping objects
# TODO 1 make local if I don't export as output position
# TODO 2 make smach viz fail; convert into ROS msgs?
Object = namedtuple('Object', ['dist', 'name', 'pose'])
PickLoc = namedtuple('PickLoc', ['size', 'name', 'objs', 'dist', 'arm_pose',
                                 'picking_pose', 'approach_pose', 'detach_pose'])
PickPlan = namedtuple('PickPlan', ['size', 'dist', 'plocs'])


class PickLocFields(smach.State):
    """
    Extract PickLoc fields on separated keys.  TODO really there's no way around this shit???
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['picking_loc'],
                             output_keys=['approach_pose', 'picking_pose', 'detach_pose'])

    def execute(self, ud):
        ud['approach_pose'] = ud['picking_loc'].approach_pose
        ud['picking_pose'] = ud['picking_loc'].picking_pose
        ud['detach_pose'] = ud['picking_loc'].detach_pose
        return 'succeeded'


class GroupObjects(smach.State):
    """
    Group detected objects reachable from picking location (within arm's reach).
    We first eliminate the locations without objects only reachable from there.
    If an object can be reached from two of the remaining locations, we choose
    the one that places the object closer to the robot arm.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_reachable_objs'],
                             input_keys=['objects', 'robot_pose', 'picking_poses'],
                             output_keys=['picking_locs'])
        self.manip_frame = rospy.get_param('~rec_objects_frame', 'arm_base_link')

    def execute(self, ud):
        bfp_to_arm_tf = Transform.create(TF2().lookup_transform('base_footprint', self.manip_frame))  # base to arm tf
        map_to_arm_tf = Transform.create(TF2().lookup_transform('map', self.manip_frame))  # map to arm tf
        map_to_fbp_tf = Transform.create(TF2().lookup_transform('map', 'base_footprint'))  # map to arm tf
        pick_locs = []
        for name, pose in ud['picking_poses'].items():
            # current distance from the robot; not used by now
            dist_from_robot = distance_2d(pose, ud['robot_pose'])
            # apply base to arm tf, so we get arm pose on map reference for each location
            # map_to_bfp_tf = Transform.create(pose)
            arm_pose_mrf = (Transform.create(pose) * bfp_to_arm_tf).to_geometry_msg_pose_stamped()
            # detected objects poses are in arm reference, so their modulo is the distance to the arm
            objs = []
            for i, obj in enumerate(ud['objects']):
                obj_pose = obj.primitive_poses[0]
                # transform object pose from arm to map frame, so we can compare distances to picking locations
                # we must limit max arm reach with our navigation tolerance when reaching the goal, as that will
                # be our probable picking pose, instead of the ideal one received as input
                ##arm_to_obj_tf = Transform.create(obj_pose)
                obj_pose_mrf = (map_to_fbp_tf * Transform.create(obj_pose)).to_geometry_msg_pose_stamped()
                dist = distance_2d(obj_pose_mrf, arm_pose_mrf)  # both on map rf
                if dist <= cfg.MAX_ARM_REACH - cfg.TIGHT_DIST_TOLERANCE:
                    objs.append(Object(dist, obj.id, obj_pose))
            if not objs:
                continue  # no objects reachable from here; keep going
            # sort objects by increasing distance from the arm; that should make picking easier,
            # as we won't hit closer objects when going over them (not 100% sure if this is true)
            objs = sorted(objs, key=lambda o: o.dist)
            # set also the approach and detach poses, at APPROACH_DIST_TO_TABLE and DETACH_DIST_FROM_TABLE respectively
            approach_pose = deepcopy(pose)
            translate_pose(approach_pose, - (cfg.APPROACH_DIST_TO_TABLE - cfg.PICKING_DIST_TO_TABLE), 'x')
            detach_pose = deepcopy(pose)
            translate_pose(detach_pose, - (cfg.DETACH_DIST_FROM_TABLE - cfg.PICKING_DIST_TO_TABLE), 'x')
            pick_locs.append(PickLoc(len(objs), name, objs, dist_from_robot, arm_pose_mrf,
                                     pose, approach_pose, detach_pose))
        if not pick_locs:
            rospy.loginfo("No reachable objects")
            return 'no_reachable_objs'

        # find the most efficient sequence of picking locations: try all permutations, sort and get the first
        possible_plans = []
        for perm in permutations(pick_locs):
            perm = list(perm)  # convert to list, so we can remove elements
            self.filter_pick_locs(perm)
            possible_plans.append(PickPlan(len(perm), self.traveled_dist(ud['robot_pose'], perm), perm))
        # sort by  1) less locations to visit  2) less travelled distance (in case of match)
        sorted_plans = sorted(possible_plans, key=lambda p: (p.size, p.dist))
        sorted_plocs = sorted_plans[0].plocs

        # remove duplicated objects from the location where they are at the longest distance to the arm
        objs_to_remove = []
        for ploc in sorted_plocs:
            dup_objs = 0
            for obj in ploc.objs:
                for other_pl in [pl for pl in sorted_plocs if pl != ploc]:
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
        for i, ploc in enumerate(sorted_plocs):
            if ploc.size != len(ploc.objs):
                sorted_plocs[i] = ploc._replace(size=len(ploc.objs))
        sorted_plocs = sorted(sorted_plocs, key=lambda pl: pl.size)  # sort again after removing duplicates
        self.viz_pick_locs(sorted_plocs)
        ud['picking_locs'] = sorted_plocs
        return 'succeeded'

    @staticmethod
    def traveled_dist(robot_pose, pick_locs):
        """
        The total distance the robot will travel from its current location after visiting all pick locations
        """
        if not pick_locs:
            return 0.0
        total_dist = distance_2d(robot_pose, pick_locs[0].picking_pose)
        for ploc1, ploc2 in zip(pick_locs, pick_locs[1:]):
            total_dist += distance_2d(ploc1.pose, ploc2.pose)
        return total_dist

    @staticmethod
    def filter_pick_locs(pick_locs):
        """
        Traverse pick locations in order, discarding those whose objects are all present in other locations
        (that is, we can skip them when picking without missing any object)
        """
        pls_to_remove = []
        for ploc in pick_locs:
            objs_in_this = {o.name for o in ploc.objs}
            objs_in_others = set()
            for other_pl in [pl for pl in pick_locs if pl.name != ploc.name and pl not in pls_to_remove]:
                objs_in_others.update(o.name for o in other_pl.objs)
            if objs_in_this.issubset(objs_in_others):
                pls_to_remove.append(ploc)
        for ploc in pls_to_remove:
            pick_locs.remove(ploc)

    def viz_pick_locs(self, pick_locs):
        Visualization().clear_markers()
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


class MakePickingPlan(smach.State):
    """
    Sort picking locations by visiting order, so we can iterate over the list for collecting all objects.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['picking_locs', 'robot_pose'],
                             output_keys=['picking_plan'])

    def execute(self, ud):
        robot_pose = ud['robot_pose']
        picking_locs = ud['picking_locs']
        picking_plan = []
        while picking_locs:
            closest_ploc = None
            closest_dist = float('inf')
            for ploc in picking_locs:
                dist = distance_2d(ploc.approach_pose, robot_pose)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_ploc = ploc
            picking_locs.remove(closest_ploc)
            picking_plan.append(closest_ploc)
            robot_pose = closest_ploc.approach_pose
            Visualization().add_text_marker(closest_ploc.approach_pose, str(len(picking_plan)) + ' ' + str(closest_dist))
            Visualization().publish_markers()
        ud['picking_plan'] = picking_plan
        return 'succeeded'


def GatherObjects(target_types):
    """
    Object gatherer SM: approach the requested table and pick all the objects of the requested type
    """

    # approach to the table
    approach_table_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                       connector_outcome='succeeded',
                                       input_keys=['table', 'table_pose'])
    with approach_table_sm:
        smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
        smach.Sequence.add('CALC_APPROACHES', TableSidesPoses(cfg.APPROACH_DIST_TO_TABLE),
                           transitions={'no_valid_table': 'aborted'},
                           remapping={'poses': 'approach_poses',
                                      'closest_pose': 'closest_approach_pose'})
        smach.Sequence.add('CALC_DETACH_POSES', TableSidesPoses(cfg.DETACH_DIST_FROM_TABLE),
                           transitions={'no_valid_table': 'aborted'},
                           remapping={'poses': 'detach_poses',
                                      'closest_pose': 'closest_detach_pose'})
        smach.Sequence.add('APPROACH_TABLE', GoToPose(),  # use default tolerances; no precision needed here
                           transitions={'aborted': 'aborted',
                                        'preempted': 'preempted'},
                           remapping={'target_pose': 'closest_approach_pose'})

    # detects objects over the table, and make a picking plan
    make_pick_plan_sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'no_reachable_objs'],
                                           input_keys=['table', 'object_types'],
                                           output_keys=['picking_poses', 'closest_picking_pose', 'picking_plan'])
    with make_pick_plan_sm:
        smach.StateMachine.add('GET_ROBOT_POSE', GetRobotPose(),
                               transitions={'succeeded': 'DETECT_TABLES'})
        smach.StateMachine.add('DETECT_TABLES', MonitorTables(2.0),  # re-detect when nearby for more precision
                               transitions={'succeeded': 'TABLE_OBSTACLE',  # (or fail if cannot see again)
                                            'aborted': 'aborted'})
        smach.StateMachine.add('TABLE_OBSTACLE', TableAsObstacle(),
                               transitions={'succeeded': 'CALC_PICK_POSES'},
                               remapping={'pose': 'table_pose'})
        smach.StateMachine.add('CALC_PICK_POSES', TableSidesPoses(cfg.PICKING_DIST_TO_TABLE),
                               transitions={'succeeded': 'DETECT_OBJECTS',
                                            'no_valid_table': 'aborted'},
                               remapping={'poses': 'picking_poses',
                                          'closest_pose': 'closest_picking_pose'})
        smach.StateMachine.add('DETECT_OBJECTS', ObjectDetection(),
                               transitions={'succeeded': 'OBJECTS_FOUND?',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('OBJECTS_FOUND?', ObjectsDetected(),
                               transitions={'true': 'GROUP_OBJECTS',
                                            'false': 'no_reachable_objs'})
        smach.StateMachine.add('GROUP_OBJECTS', GroupObjects(),
                               transitions={'succeeded': 'MAKE_PICKING_PLAN'})
        smach.StateMachine.add('MAKE_PICKING_PLAN', MakePickingPlan())

    # go to a picking location and pick all objects reachable from there
    pick_objects_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted', 'tray_full'],
                                     connector_outcome='succeeded',
                                     input_keys=['table', 'picking_loc', 'object_types'])

    with pick_objects_sm:
        smach.Sequence.add('PICK_LOC_FIELDS', PickLocFields())
        smach.Sequence.add('GOTO_APPROACH', GoToPose(),  # use default tolerances; no precision needed here
                           transitions={'aborted': 'aborted',
                                        'preempted': 'preempted'},
                           remapping={'target_pose': 'approach_pose'})
        smach.Sequence.add('ALIGN_TO_TABLE', AlignToTable(),
                           transitions={'aborted': 'aborted',
                                        'preempted': 'preempted'},
                           remapping={'pose': 'picking_pose'})
        smach.Sequence.add('PICK_OBJECTS', PickReachableObjs())
        smach.Sequence.add('DETACH_FROM_TABLE', DetachFromTable(),
                           remapping={'pose': 'detach_pose'})

    # iterate over all picking locations in the plan
    pick_objects_it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted', 'tray_full'],
                                     input_keys=['table', 'picking_plan', 'object_types'],
                                     output_keys=[],
                                     it=lambda: pick_objects_it.userdata.picking_plan,
                                     it_label='picking_loc',
                                     exhausted_outcome='succeeded')
    with pick_objects_it:
        smach.Iterator.set_contained_state('', pick_objects_sm, loop_outcomes=['succeeded'])

    # Full SM: approach the table, make a picking plan and execute it, double-checking that we left no object behind
    sm = DoOnExitContainer(outcomes=['succeeded',
                                     'aborted',
                                     'preempted',
                                     'tray_full'],
                           input_keys=['table', 'table_pose'])
    sm.userdata.object_types = target_types
    with sm:
        smach.StateMachine.add('PRECISE_CTRL', SetNamedConfig('precise_controlling'),
                               transitions={'succeeded': 'APPROACH_TABLE',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('APPROACH_TABLE', approach_table_sm,
                               transitions={'succeeded': 'MAKE_PICK_PLAN',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('MAKE_PICK_PLAN', make_pick_plan_sm,
                               transitions={'succeeded': 'PICK_OBJECTS',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted',
                                            'no_reachable_objs': 'succeeded'})
        smach.StateMachine.add('PICK_OBJECTS', pick_objects_it,
                               transitions={'succeeded': 'DETECT_OBJECTS',  # double-check that we left no object behind
                                            'aborted': 'aborted',     # restore original configuration on error
                                            'preempted': 'preempted',
                                            'tray_full': 'tray_full'})
        smach.StateMachine.add('DETECT_OBJECTS', ObjectDetection(),
                               transitions={'succeeded': 'OBJECTS_LEFT',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('OBJECTS_LEFT', ObjectsDetected(),
                               transitions={'true': 'APPROACH_TABLE',   # at least one object left; restart picking
                                            'false': 'succeeded'})      # otherwise we are done
        DoOnExitContainer.add_finally('STANDARD_CTRL', DismissNamedConfig('precise_controlling'))
    return sm
