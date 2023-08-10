import rospy
import smach
import smach_ros

from thorp_toolkit.geometry import TF2, to_transform, transform_pose, create_2d_pose, distance_2d
from thorp_msgs.msg import PlanPickingAction

from .costmaps import TableAsObstacle
from .semantics import TableMarkVisited
from .perception import MonitorTables, ObjectDetection, ObjectsDetected, ClearMarkers
from .navigation import GetRobotPose, AreSamePose, GoToPose, AlignToTable, DetachFromTable
from .manipulation import ClearPlanningScene
from .pickup_objs import PickupReachableObjs
from ..containers.do_on_exit import DoOnExit as DoOnExitContainer

from .. import config as cfg


class MakePickingPlan(smach_ros.SimpleActionState):
    def __init__(self):
        super(MakePickingPlan, self).__init__('manipulation/plan_picking',
                                              PlanPickingAction,
                                              goal_cb=self.make_goal,
                                              goal_slots=['robot_pose', 'objects', 'surface'],
                                              result_cb=self.result_cb,
                                              output_keys=['picking_plan'])

    def make_goal(self, ud, goal):
        goal.planning_frame = cfg.PICKING_PLANNING_FRAME
        goal.approach_dist = cfg.APPROACH_DIST_TO_TABLE
        goal.picking_dist = cfg.PICKING_DIST_TO_TABLE
        goal.detach_dist = cfg.DETACH_DIST_FROM_TABLE
        goal.max_arm_reach = cfg.MAX_ARM_REACH - cfg.TIGHT_DIST_TOLERANCE

    def result_cb(self, ud, status, result):
        ud['picking_plan'] = result.picking_plan.locations


class ClosestSidePose(smach.State):
    """
    Calculate the four locations around a segmented surface at a given distance and return the closest to the robot.
    """

    def __init__(self, distance):
        smach.State.__init__(self, outcomes=['succeeded', 'no_valid_table'],
                             input_keys=['table', 'table_pose', 'robot_pose'],
                             output_keys=['pose'])
        self.distance = distance

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
        poses = [create_2d_pose(p_x, 0.0, -pi, 'table_frame'),
                 create_2d_pose(n_x, 0.0, 0.0, 'table_frame'),
                 create_2d_pose(0.0, p_y, -pi / 2.0, 'table_frame'),
                 create_2d_pose(0.0, n_y, +pi / 2.0, 'table_frame')]
        for pose in poses:
            pose = transform_pose(pose, table_tf)
            dist = distance_2d(pose, ud['robot_pose'])
            if dist < closest_dist:
                closest_pose = pose
                closest_dist = dist
        ud['pose'] = closest_pose
        return 'succeeded'


class PickLocFields(smach.State):
    """
    Extract PickLoc fields on separated keys.
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


class CountGivenUpObjects(smach.State):
    """
    Increase given up objects count with the failures from last picking spot
    """

    def __init__(self):
        super(CountGivenUpObjects, self).__init__(outcomes=['succeeded'],
                                                  input_keys=['given_up_count', 'failures'],
                                                  output_keys=['given_up_count'])

    def execute(self, ud):
        ud['given_up_count'] += sum(1 for fc in ud['failures'].values() if fc == cfg.PICKING_MAX_FAILURES)
        return 'succeeded'


class ThereAreObjectsLeft(smach.State):
    """
    Check whether we have more objects left than those given up
    """

    def __init__(self):
        super(ThereAreObjectsLeft, self).__init__(outcomes=['true', 'false'],
                                                  input_keys=['objects', 'given_up_count'])

    def execute(self, ud):
        objects_count = len(ud['objects']) - ud['given_up_count']
        return 'true' if objects_count > 0 else 'false'


class GatherObjects(smach.StateMachine):
    """
    Object gatherer SM: approach the requested table and pick all the objects of the requested type
    """

    def __init__(self, target_types):
        super(GatherObjects, self).__init__(outcomes=['succeeded', 'aborted', 'preempted', 'tray_full'],
                                            input_keys=['table', 'table_pose'])

        self.userdata.object_types = target_types

        # approach to the table
        approach_table_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                           connector_outcome='succeeded',
                                           input_keys=['table', 'table_pose'])
        with approach_table_sm:
            smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
            smach.Sequence.add('CALC_APPROACH', ClosestSidePose(cfg.APPROACH_DIST_TO_TABLE),
                               transitions={'no_valid_table': 'aborted'},
                               remapping={'pose': 'closest_approach_pose'})
            smach.Sequence.add('APPROACH_TABLE', GoToPose(),  # use default tolerances; no precision needed here
                               transitions={'aborted': 'aborted',
                                            'preempted': 'preempted'},
                               remapping={'target_pose': 'closest_approach_pose'})

        # detects objects over the table, and make a picking plan
        make_picking_plan_sm = DoOnExitContainer(outcomes=['succeeded', 'aborted', 'preempted', 'no_reachable_objs'],
                                                 input_keys=['table', 'table_pose', 'object_types'],
                                                 output_keys=['closest_picking_pose', 'picking_plan'])
        with make_picking_plan_sm:
            smach.StateMachine.add('GET_ROBOT_POSE', GetRobotPose(),
                                   transitions={'succeeded': 'PICKING_POSE'})
            smach.StateMachine.add('PICKING_POSE', ClosestSidePose(cfg.PICKING_DIST_TO_TABLE),
                                   transitions={'succeeded': 'ALIGN_TO_TABLE',
                                                'no_valid_table': 'aborted'},
                                   remapping={'pose': 'closest_picking_pose'})
            smach.StateMachine.add('ALIGN_TO_TABLE', AlignToTable(),
                                   transitions={'succeeded': 'DETECT_OBJECTS',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'},
                                   remapping={'pose': 'closest_picking_pose'})
            smach.StateMachine.add('DETECT_OBJECTS', ObjectDetection(),
                                   transitions={'succeeded': 'TABLE_VISITED',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})
            smach.StateMachine.add('TABLE_VISITED', TableMarkVisited(),
                                   transitions={'succeeded': 'TABLE_OBSTACLE'})
            smach.StateMachine.add('TABLE_OBSTACLE', TableAsObstacle(),
                                   transitions={'succeeded': 'OBJECTS_FOUND?'},
                                   remapping={'pose': 'table_pose'})
            smach.StateMachine.add('OBJECTS_FOUND?', ObjectsDetected(),
                                   transitions={'true': 'MAKE_PICKING_PLAN',
                                                'false': 'no_reachable_objs'})
            smach.StateMachine.add('MAKE_PICKING_PLAN', MakePickingPlan())
            DoOnExitContainer.add_finally('CLEAR_P_SCENE', ClearPlanningScene())

        # go to a picking location and pick all objects reachable from there
        pickup_objects_sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'tray_full'],
                                               input_keys=['given_up_count', 'table', 'picking_loc', 'object_types'],
                                               output_keys=['given_up_count'])

        with pickup_objects_sm:
            smach.StateMachine.add('PICK_LOC_FIELDS', PickLocFields(),
                                   transitions={'succeeded': 'GET_ROBOT_POSE'})
            smach.StateMachine.add('GET_ROBOT_POSE', GetRobotPose(),
                                   transitions={'succeeded': 'AT_PICKING_POSE?'})
            smach.StateMachine.add('AT_PICKING_POSE?', AreSamePose(),
                                   transitions={'true': 'PICK_OBJECTS',
                                                'false': 'GOTO_APPROACH'},
                                   remapping={'pose1': 'robot_pose',
                                              'pose2': 'picking_pose'})
            smach.StateMachine.add('GOTO_APPROACH', GoToPose(),  # use default tolerances; no precision needed here
                                   transitions={'succeeded': 'ALIGN_TO_TABLE'},
                                   remapping={'target_pose': 'approach_pose'})
            smach.StateMachine.add('ALIGN_TO_TABLE', AlignToTable(),
                                   transitions={'succeeded': 'PICK_OBJECTS'},
                                   remapping={'pose': 'picking_pose'})
            smach.StateMachine.add('PICK_OBJECTS', PickupReachableObjs(),
                                   transitions={'succeeded': 'COUNT_GIVEN_UP'})
            smach.StateMachine.add('COUNT_GIVEN_UP', CountGivenUpObjects(),
                                   transitions={'succeeded': 'CLEAR_MARKERS'})
            smach.StateMachine.add('CLEAR_MARKERS', ClearMarkers(),
                                   transitions={'succeeded': 'DETACH_FROM_TABLE',
                                                'aborted': 'DETACH_FROM_TABLE'})
            smach.StateMachine.add('DETACH_FROM_TABLE', DetachFromTable(),
                                   remapping={'pose': 'detach_pose'})

        # iterate over all picking locations in the plan
        pickup_objects_it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted', 'tray_full'],
                                           input_keys=['given_up_count', 'table', 'picking_plan', 'object_types'],
                                           output_keys=['given_up_count'],
                                           it=lambda: pickup_objects_it.userdata.picking_plan,
                                           it_label='picking_loc',
                                           exhausted_outcome='succeeded')
        with pickup_objects_it:
            smach.Iterator.set_contained_state('', pickup_objects_sm, loop_outcomes=['succeeded'])

        # Full SM: approach the table, make a picking plan and execute it, double-checking that we left no object behind
        with self:
            smach.StateMachine.add('APPROACH_TABLE', approach_table_sm,
                                   transitions={'succeeded': 'RE_DETECT_TABLE',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})
            smach.StateMachine.add('RE_DETECT_TABLE', MonitorTables(2.0),
                                   transitions={'succeeded': 'MAKE_PICK_PLAN',
                                                # re-detect when nearby for more precision,
                                                'aborted': 'succeeded'})  # or just succeed to give up if not seen again
            smach.StateMachine.add('MAKE_PICK_PLAN', make_picking_plan_sm,
                                   transitions={'succeeded': 'EXEC_PICK_PLAN',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted',
                                                'no_reachable_objs': 'succeeded'})
            smach.StateMachine.add('EXEC_PICK_PLAN', pickup_objects_it,
                                   transitions={'succeeded': 'DETECT_OBJECTS',
                                                # double-check that we left no object behind
                                                'aborted': 'aborted',  # restore original configuration on error
                                                'preempted': 'preempted',
                                                'tray_full': 'tray_full'})
            smach.StateMachine.add('DETECT_OBJECTS', ObjectDetection(),
                                   transitions={'succeeded': 'OBJECTS_LEFT?',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})
            smach.StateMachine.add('OBJECTS_LEFT?', ThereAreObjectsLeft(),
                                   transitions={'true': 'APPROACH_TABLE',  # at least one object left; restart picking
                                                'false': 'succeeded'})  # otherwise, we are done

    def execute(self, parent_ud=smach.UserData()):
        self.userdata.given_up_count = 0  # keep track of given up objects on current table
        return super(GatherObjects, self).execute(parent_ud)
