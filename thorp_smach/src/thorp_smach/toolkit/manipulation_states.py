import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import control_msgs.msg as control_msgs
import geometry_msgs.msg as geometry_msgs
from turtlebot_arm_block_manipulation.msg import BlockDetectionAction

from copy import deepcopy
from collections import namedtuple
from visualization_msgs.msg import Marker, MarkerArray

from thorp_toolkit.planning_scene import PlanningScene
from thorp_toolkit.geometry import TF2, to_transform, transform_pose, apply_transform,\
                                   create_2d_pose, create_3d_pose, pose2d2str, distance_2d
from thorp_toolkit.visualization import Visualization

import config as cfg


def FoldArm():
    """ Concurrently fold arm and close the gripper """
    sm = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                           default_outcome='succeeded',
                           outcome_map={'succeeded': {'CLOSE_GRIPPER': 'succeeded',
                                                      'GOTO_RESTING': 'succeeded'},
                                        'preempted': {'CLOSE_GRIPPER': 'preempted',
                                                      'GOTO_RESTING': 'preempted'},
                                        'aborted': {'CLOSE_GRIPPER': 'aborted',
                                                    'GOTO_RESTING': 'aborted'}})
    with sm:
        smach.Concurrence.add('CLOSE_GRIPPER',
                              smach_ros.SimpleActionState('gripper_controller/gripper_action',
                                                          control_msgs.GripperCommandAction,
                                                          goal=control_msgs.GripperCommandGoal(
                                                              control_msgs.GripperCommand(0.025, 0.0))))
        smach.Concurrence.add('GOTO_RESTING',
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


def PickupObject(attempts=1):
    """  Pickup a given object, optionally retrying up to a given number of times  """
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
            smach.StateMachine.add('PICKUP_OBJECT',
                                   smach_ros.SimpleActionState('pickup_object',
                                                               thorp_msgs.PickupObjectAction,
                                                               goal_slots=['object_name', 'support_surf', 'max_effort'],
                                                               result_slots=[]),
                                   transitions={'succeeded': 'succeeded',
                                                'preempted': 'preempted',
                                                'aborted': 'CLEAR_OCTOMAP'})

            smach.StateMachine.add('CLEAR_OCTOMAP',
                                   smach_ros.ServiceState('clear_octomap',
                                                          std_srvs.Empty),
                                   transitions={'succeeded': 'continue',
                                                'preempted': 'preempted',
                                                'aborted': 'aborted'})

        # TODOs:
        #  - we should open the gripper, in case we have picked an object
        #  - check error and, if collision between parts of the arm, move a bit the arm  -->  not enough info
        #  - this doesn't make too much sense as a loop... better try all our tricks and exit

        smach.Iterator.set_contained_state('', sm, loop_outcomes=['continue'])

    return it


def PlaceObject(attempts=1):
    """  Place a given object, optionally retrying up to a given number of times  """
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
            smach.StateMachine.add('PLACE_OBJECT',
                                   smach_ros.SimpleActionState('place_object',
                                                               thorp_msgs.PlaceObjectAction,
                                                               goal_slots=['object_name', 'support_surf', 'place_pose'],
                                                               result_slots=[]),
                                   transitions={'succeeded': 'succeeded',
                                                'preempted': 'preempted',
                                                'aborted': 'CLEAR_OCTOMAP'})

            smach.StateMachine.add('CLEAR_OCTOMAP',
                                   smach_ros.ServiceState('clear_octomap',
                                                          std_srvs.Empty),
                                   transitions={'succeeded': 'continue',
                                                'preempted': 'preempted',
                                                'aborted': 'aborted'})

        # TODOs:
        #  - we should open the gripper, in case we have picked an object
        #  - check error and, if collision between parts of the arm, move a bit the arm  -->  not enough info
        #  - this doesn't make too much sense as a loop... better try all our tricks and exit

        smach.Iterator.set_contained_state('', sm, loop_outcomes=['continue'])

    return it


def PlaceInTray():
    """  Place a given object on the tray  """
    sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted', 'tray_full'],
                        connector_outcome='succeeded',
                        input_keys=['object_name'])
    sm.userdata.support_surf = 'tray'
    with sm:
        smach.Sequence.add('POSE_IN_TRAY', GetPoseInTray())
        smach.Sequence.add('PLACE_ON_TRAY', PlaceObject(),
                           remapping={'place_pose': 'pose_in_tray'})
        smach.Sequence.add('READJUST_POSE', DisplaceObject(),
                           remapping={'new_pose': 'pose_in_tray'})

    return sm


class RemoveObject(smach.State):
    """
    Remove collision object from the planning scene
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['object_name'])

    def execute(self, ud):
        rospy.loginfo("Removing from scene object '%s' placed on tray", ud['object_name'])
        PlanningScene().remove_obj(ud['object_name'])
        return 'succeeded'


class DisplaceObject(smach.State):
    """
    Displace a collision object in the planning scene
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['object_name', 'new_pose'])

    def execute(self, ud):
        new_pose = ud['new_pose']
        new_pose.pose.position.z -= cfg.PLACING_HEIGHT_ON_TRAY  # undo added clearance to replicate gravity
        rospy.loginfo("Object '%s' pose in tray readjusted to %s", ud['object_name'], pose2d2str(new_pose))
        PlanningScene().displace_obj(ud['object_name'], ud['new_pose'])
        return 'succeeded'


class GetPoseInTray(smach.State):
    """
    Calculate the next pose within the tray to use.
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'tray_full'],
                             output_keys=['pose_in_tray'])
        self.tray_link = rospy.get_param('tray_link', 'tray_link')
        self.tray_full = False
        self.slots_x = int(cfg.TRAY_SIDE_X / cfg.TRAY_SLOT + 0.1)  # avoid float division pitfall
        self.slots_y = int(cfg.TRAY_SIDE_Y / cfg.TRAY_SLOT + 0.1)  # until I switch to Python3
        self.offset_x = 0.0 if self.slots_x % 2 else cfg.TRAY_SLOT / 2.0
        self.offset_y = 0.0 if self.slots_y % 2 else cfg.TRAY_SLOT / 2.0
        self.next_x = 0
        self.next_y = 0

        # visualize place poses (for debugging)
        points = []
        for _ in range(self.slots_x * self.slots_y):
            points.append(self._next_pose().pose.position)
        Visualization().add_markers(Visualization().create_point_list(create_2d_pose(0, 0, 0, self.tray_link),
                                                                      points, [1, 0, 0, 1]))  # solid red points
        Visualization().publish_markers()
        self.tray_full = False

    def execute(self, ud):
        if self.tray_full:
            return 'tray_full'

        # add a collision object for the tray surface, right above the mesh
        PlanningScene().add_tray(create_3d_pose(0, 0, 0.0015, 0, 0, 0, self.tray_link),
                                 (cfg.TRAY_SIDE_X, cfg.TRAY_SIDE_Y, 0.002))
        ud['pose_in_tray'] = self._next_pose()
        return 'succeeded'

    def _next_pose(self):
        # Get next empty location coordinates
        x = (self.next_x - self.slots_x/2) * cfg.TRAY_SLOT + self.offset_x
        y = (self.next_y - self.slots_y/2) * cfg.TRAY_SLOT + self.offset_y
        z = 0.0125  # TODO: should be obj.size.z/2; replace once I have proper object recognition
        z += cfg.PLACING_HEIGHT_ON_TRAY  # place objects 3cm above the tray, so they fall into position
        self.next_x = (self.next_x + 1) % self.slots_x
        if self.next_x == 0:
            self.next_y = (self.next_y + 1) % self.slots_y
            if self.next_x == self.next_y == 0:
                self.tray_full = True
        return create_3d_pose(x, y, z, 0, 0, 0, self.tray_link)


def GatherBlocks():
    """  Pick all the objects in the list a place in the tray  """

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


Object = namedtuple('Object', ['dist', 'name', 'pose'])
PickLoc = namedtuple('PickLoc', ['size', 'name', 'pose', 'arm_pose', 'objs'])

class GroupObjects(smach.State):
    """
    Group detected objects reachable from picking location (within the given distance from the arm).
    If an object can be reached from two locations that we need to
    visit, choose the one that makes the object closer to the robot.
    """

    def __init__(self, distance, manip_frame):
        smach.State.__init__(self, outcomes=['succeeded', 'no_objects'],
                             input_keys=['objects', 'table', 'picking_poses'],
                             output_keys=['picking_plan'])
        self.max_pick_dist = distance
        self.manip_frame = manip_frame
        self.groups_viz = rospy.Publisher('manipulation/picking_groups', MarkerArray, queue_size=1)

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
                if dist <= self.max_pick_dist:
                    objs.append(Object(dist, 'block' + str(i + 1), obj_pose))
            objs = sorted(objs, key=lambda o: o.dist)  # sort by increasing distance from the arm
            pick_locs.append(PickLoc(len(objs), name, pose, arm_pose_mrf, objs))
        sorted_pls = sorted(pick_locs, key=lambda pl: pl.size)  # sort by increasing number of reachable objects
        self.viz_pick_locs(sorted_pls)

        # traverse pick locations from less to more populated, and discard those containing only duplicated objects
        # (or no objects at all), so we can skip them
        pls_to_remove = []
        for pl in sorted_pls:
            dup_objs = 0
            for obj in pl.objs:
                for other_pl in [pl for pl in sorted_pls if pl not in pls_to_remove]:
                    if other_pl == pl:
                        continue
                    if obj.name in [obj.name for obj in other_pl.objs]:
                        dup_objs += 1
            if dup_objs == pl.size:
                pls_to_remove.append(pl)
        for pl in pls_to_remove:
            sorted_pls.remove(pl)
        self.viz_pick_locs(sorted_pls)

        # remove duplicated objects from the location where they are at the longest distance to the arm
        objs_to_remove = []
        for pl in sorted_pls:
            dup_objs = 0
            for obj in pl.objs:
                for other_pl in sorted_pls:
                    if other_pl == pl:
                        continue
                    for other_obj in other_pl.objs:
                        if obj.name == other_obj.name:
                            if obj.dist >= other_obj.dist:
                                objs_to_remove.append((pl, obj))
                            else:
                                objs_to_remove.append((other_pl, other_obj))
                            dup_objs += 1
        for pl, obj in objs_to_remove:
            pl.objs.remove(obj)
        self.viz_pick_locs(sorted_pls)
        ud['picking_plan'] = sorted_pls

        return 'succeeded'

    def viz_pick_locs(self, pick_locs):
        # TODO use Visualization()
        ma = MarkerArray()
        for pl in pick_locs:
            dm = Marker()
            dm.header = pl.arm_pose.header
            dm.ns = pl.name + ' dist'
            dm.type = Marker.CYLINDER
            dm.action = Marker.ADD
            dm.lifetime = rospy.Duration(60)
            dm.scale.x = self.max_pick_dist * 2.0
            dm.scale.y = self.max_pick_dist * 2.0
            dm.scale.z = 0.001
            dm.color = Visualization.rand_color(0.2)
            dm.pose = deepcopy(pl.arm_pose.pose)
            ma.markers.append(dm)

            nm = deepcopy(dm)
            nm.ns = pl.name + ' name'
            nm.type = Marker.TEXT_VIEW_FACING
            nm.text = pl.name + ' ' + str(pl.size)
            nm.scale.x = 0.0
            nm.scale.y = 0.0
            nm.scale.z = 0.2
            nm.color.a = 0.5
            nm.pose = deepcopy(pl.arm_pose.pose)
            nm.pose.position.z += 0.15
            ma.markers.append(nm)
            for obj in pl.objs:
                om = deepcopy(nm)
                om.header.frame_id = self.manip_frame
                om.ns = pl.name + ' ' + obj.name
                om.text = obj.name
                om.scale.z = 0.1
                om.pose = deepcopy(obj.pose)
                om.pose.position.z += 0.05
                ma.markers.append(om)

        self.groups_viz.publish(ma)
