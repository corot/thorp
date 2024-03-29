import rospy
import smach
import smach_ros

import nav_msgs.msg as nav_msgs
import mbf_msgs.msg as mbf_msgs
import thorp_msgs.msg as thorp_msgs
import thorp_msgs.srv as thorp_srvs

from thorp_toolkit.geometry import TF2, pose2d2str, heading, quaternion_msg_from_yaw, same_pose
from thorp_toolkit.reconfigure import Reconfigure
from thorp_toolkit.progress_tracker import ProgressTracker

from .common import SetNamedConfig, DismissNamedConfig
from .userdata import UDInsertInList, UDListSlicing, UDApplyFn
from ..states.costmaps import ClearTableWay, RestoreTableWay
from ..containers.do_on_exit import DoOnExit as DoOnExitContainer

Reconfigure().load_named_configs()  # load named configurations from the default location


class GetRobotPose(smach.State):
    """ Add current robot pose to ud """

    def __init__(self):
        super(GetRobotPose, self).__init__(outcomes=['succeeded', 'aborted'],
                                           output_keys=['robot_pose'])

    def execute(self, ud):
        try:
            ud['robot_pose'] = TF2().transform_pose(None, 'base_footprint', 'map')
            return 'succeeded'
        except rospy.ROSException as err:
            rospy.logerr("Get robot pose failed: %s", str(err))
            return 'aborted'


class AreSamePose(smach.State):
    """
    Check whether two poses are the same (within tolerance thresholds)
    """

    def __init__(self):
        super(AreSamePose, self).__init__(outcomes=['true', 'false'],
                                          input_keys=['pose1', 'pose2'])

    def execute(self, ud):
        return 'true' if same_pose(ud['pose1'], ud['pose2'], xy_tolerance=0.1, yaw_tolerance=0.1) else 'false'


class PrependCurrentPose(smach.Sequence):
    """
    Prepend current robot pose to the given path.
    Normally we do this to ensure that we follow the whole path without skipping to the nearest pose.
    This also helps progress tracker when the first waypoint is already behind.
    """

    def __init__(self):
        super(PrependCurrentPose, self).__init__(outcomes=['succeeded', 'aborted'],
                                                 connector_outcome='succeeded',
                                                 input_keys=['path'],
                                                 output_keys=['path'])
        with self:
            smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
            smach.Sequence.add('INSERT_CURRENT_POSE', UDInsertInList(0),
                               remapping={'element': 'robot_pose',
                                          'list': 'path'})


class DelTraversedWPs(smach.Sequence):
    """
    Deleted waypoints up to the last reached one.
    """

    def __init__(self):
        super(DelTraversedWPs, self).__init__(outcomes=['succeeded', 'aborted'],
                                              connector_outcome='succeeded',
                                              input_keys=['waypoints', 'reached_wp'],
                                              output_keys=['waypoints'])
        with self:
            smach.Sequence.add('INC_REACHED_WP_BY_1', UDApplyFn('reached_wp', lambda x: None if x is None else x + 1))
            smach.Sequence.add('DEL_TRAVERSED_WPS', UDListSlicing(),
                               remapping={'list': 'waypoints',
                                          'start': 'reached_wp'})


class PoseAsPath(smach.State):
    """ Add a path to the ud containing just the input pose """

    def __init__(self):
        super(PoseAsPath, self).__init__(outcomes=['succeeded'],
                                         input_keys=['pose'],
                                         output_keys=['path'])

    def execute(self, ud):
        ud['path'] = nav_msgs.Path(ud['pose'].header, [ud['pose']])
        return 'succeeded'


class PosesAsPath(smach.State):
    """ Add a path to the ud containing the list of input poses """

    def __init__(self):
        super(PosesAsPath, self).__init__(outcomes=['succeeded'],
                                          input_keys=['poses'],
                                          output_keys=['path'])

    def execute(self, ud):
        assert ud['poses'], "PosesAsPath: poses list is empty"
        ud['path'] = nav_msgs.Path(ud['poses'][0].header, ud['poses'])
        return 'succeeded'


class GoToPose(smach_ros.SimpleActionState):
    def __init__(self,
                 planner=None,
                 controller=None,
                 rec_behaviors=None,
                 dist_tolerance=None,
                 angle_tolerance=None):
        super(GoToPose, self).__init__('move_base_flex/move_base',
                                       mbf_msgs.MoveBaseAction,
                                       goal_cb=self.make_goal,
                                       goal_slots=['target_pose'],
                                       result_cb=self.result_cb,
                                       result_slots=['outcome', 'message'])
        self.planner = planner or rospy.get_param('~default_planner')
        self.controller = controller or rospy.get_param('~default_controller')
        self.rec_behaviors = rec_behaviors or rospy.get_param('~move_base_recovery')
        self.dist_tolerance = dist_tolerance
        self.angle_tolerance = angle_tolerance
        self.params_ns = 'move_base_flex/' + self.controller

    def make_goal(self, ud, goal):
        if self.planner:
            goal.planner = self.planner
        if self.controller:
            goal.controller = self.controller
        if self.rec_behaviors:
            goal.recovery_behaviors = self.rec_behaviors
        if self.dist_tolerance and self.angle_tolerance:
            # Set configured tolerance values
            Reconfigure().update_config(self.params_ns, {'xy_goal_tolerance': self.dist_tolerance,
                                                         'yaw_goal_tolerance': self.angle_tolerance})

    def result_cb(self, ud, status, result):
        if self.dist_tolerance and self.angle_tolerance:
            # Restore previous tolerance values before leaving the state
            Reconfigure().restore_config(self.params_ns, ['xy_goal_tolerance', 'yaw_goal_tolerance'])


class ExePath(smach_ros.SimpleActionState):
    def __init__(self, controller=None, dist_tolerance=None, angle_tolerance=None, track_progress=False):
        super(ExePath, self).__init__('move_base_flex/exe_path',
                                      mbf_msgs.ExePathAction,
                                      goal_cb=self.make_goal,
                                      result_cb=self.result_cb,
                                      result_slots=['outcome', 'message'],
                                      input_keys=['path', 'waypoints'],
                                      output_keys=['reached_wp'])
        self.controller = controller or rospy.get_param('~default_controller')
        self.dist_tolerance = dist_tolerance
        self.angle_tolerance = angle_tolerance
        self.track_progress = track_progress
        self.progress_tracker = None
        self.params_ns = 'move_base_flex/' + self.controller

    def make_goal(self, ud, goal):
        goal.path = ud['path']
        if self.controller:
            goal.controller = self.controller
        if self.dist_tolerance and self.angle_tolerance:
            # Set configured tolerance values
            Reconfigure().update_config(self.params_ns, {'xy_goal_tolerance': self.dist_tolerance,
                                                         'yaw_goal_tolerance': self.angle_tolerance})

        if self.track_progress:
            wp_reached_threshold = rospy.get_param('~wp_reached_threshold')
            rospy.loginfo("Progress tracker initialized with %d waypoints and reached threshold %g m",
                          len(ud['waypoints']), wp_reached_threshold)
            self.progress_tracker = ProgressTracker(ud['waypoints'], wp_reached_threshold)

    def result_cb(self, ud, status, result):
        if self.dist_tolerance and self.angle_tolerance:
            # Restore previous tolerance values before leaving the state
            Reconfigure().restore_config(self.params_ns, ['xy_goal_tolerance', 'yaw_goal_tolerance'])
        if self.track_progress:
            reached_waypoint = self.progress_tracker.reached_waypoint()
            if reached_waypoint is None:
                rospy.loginfo("Progress tracker didn't reach even the first waypoint")
            else:
                rospy.loginfo("Progress tracker reached waypoint: %d", reached_waypoint)
            ud['reached_wp'] = reached_waypoint
        if self.track_progress:
            self.progress_tracker.reset()  # to clear the markers, but... viz is a singleton, so I'll clear ALL markers!

    def _goal_feedback_cb(self, feedback):
        super(ExePath, self)._goal_feedback_cb(feedback)
        if self.track_progress:
            self.progress_tracker.update_pose(feedback.current_pose)


class LookToPose(ExePath):
    """ Turn toward a given pose """

    def __init__(self):
        super(LookToPose, self).__init__()
        super(ExePath, self).register_input_keys(['robot_pose', 'target_pose'])  # extend ExePath inputs

    # this decorator is the only way I found to add additional input keys (normally is used on CBState)
    @smach.cb_interface(input_keys=['robot_pose', 'target_pose'])
    def make_goal(ud, goal):
        # Create a single-pose path with the current robot location but heading toward the target pose
        heading_to_target = heading(ud['robot_pose'].pose, ud['target_pose'].pose)
        import copy
        pose = copy.deepcopy(ud['robot_pose'])
        pose.pose.orientation = quaternion_msg_from_yaw(heading_to_target)
        goal.path = nav_msgs.Path(pose.header, [pose])
        goal.controller = rospy.get_param('~default_controller')


class Recovery(smach_ros.SimpleActionState):
    def __init__(self):
        super(Recovery, self).__init__('move_base_flex/recovery',
                                       mbf_msgs.RecoveryAction,
                                       goal_slots=['behavior'],
                                       result_slots=['outcome', 'message'])


class ExePathFailed(smach.State):
    """ Handle failures on ExePath state """

    def __init__(self, recovery_behaviors=None):
        super(ExePathFailed, self).__init__(outcomes=['recover', 'next_wp', 'aborted'],
                                            input_keys=['path', 'waypoints', 'reached_wp', 'outcome', 'message'],
                                            output_keys=['path', 'waypoints', 'next_wp', 'recovery_behavior'])
        self.recovery_behaviors = recovery_behaviors or rospy.get_param('~exe_path_recovery')
        self.consecutive_failures = 0

    def execute(self, ud):
        # Cut path up to current waypoint, so we don't redo it from the beginning after recovering

        # ExePath must provide this specific message at the end of the result message in case of failure
        cwp_index = ud['message'].find('current waypoint: ')
        if cwp_index >= 0:
            # TODO I must adapt this to integrate the progress tracker with the smoothed path logic (or remove entirely path follower support)   and remove 'path' key
            next_waypoint = int(ud['message'][cwp_index + len('current waypoint: '):])
            ud['path'].poses = ud['path'].poses[next_waypoint:]
        elif ud['reached_wp'] is not None:
            # TEB logic using progress tracker: very brittle, but seems to work!  hope I can do better w/ BTs
            next_waypoint = ud['reached_wp'] + 1
            ud['waypoints'] = ud['waypoints'][next_waypoint:]
        else:
            # None on 'reached_wp' normally means a failure after a recovery attempt
            rospy.logwarn("No current waypoint provided; assuming we didn't reach even the first waypoint")

        try:
            rb = self.recovery_behaviors[self.consecutive_failures]
            ud['recovery_behavior'] = rb
            self.consecutive_failures += 1
            rospy.loginfo("Attempt recovery behavior '%s' after %d %s", rb, self.consecutive_failures,
                          "consecutive failures" if self.consecutive_failures > 1 else "failure")
            return 'recover'
        except IndexError:
            rospy.loginfo("Recovery behaviors exhausted after %d consecutive failures", self.consecutive_failures)
            self.consecutive_failures = 0
            if ud['waypoints']:
                next_wp_pose = ud['waypoints'][0]
                rospy.loginfo("Navigate to the next waypoint at %s", pose2d2str(next_wp_pose))
                ud['next_wp'] = next_wp_pose
                return 'next_wp'

            return 'aborted'


class AlignToTable(DoOnExitContainer):
    def __init__(self):
        super(AlignToTable, self).__init__(outcomes=['succeeded', 'aborted', 'preempted'],
                                           input_keys=['table', 'pose'],
                                           output_keys=['outcome', 'message'])
        with self:
            self.userdata['behavior'] = 'out_to_free_space'
            DoOnExitContainer.add('CLEAR_WAY', ClearTableWay(),
                                  transitions={'succeeded': 'PRECISE_CTRL'})
            DoOnExitContainer.add('PRECISE_CTRL', SetNamedConfig('precise_controlling'),
                                  transitions={'succeeded': 'POSE_AS_PATH',
                                               'aborted': 'aborted'})
            DoOnExitContainer.add('POSE_AS_PATH', PoseAsPath(),
                                  transitions={'succeeded': 'ALIGN_TO_TABLE'})
            DoOnExitContainer.add('ALIGN_TO_TABLE', ExePath(),
                                  transitions={'succeeded': 'succeeded',
                                               'aborted': 'TO_FREE_SPACE',
                                               'preempted': 'preempted'})
            DoOnExitContainer.add('TO_FREE_SPACE', Recovery(),
                                  transitions={'succeeded': 'CLEAR_WAY',  # retry  TODO: potential inf loop!
                                               'aborted': 'aborted',
                                               'preempted': 'preempted'})
            DoOnExitContainer.add_finally('RESTORE_WAY', RestoreTableWay())
            DoOnExitContainer.add_finally('STANDARD_CTRL', DismissNamedConfig('precise_controlling'))


class DetachFromTable(DoOnExitContainer):
    def __init__(self):
        super(DetachFromTable, self).__init__(outcomes=['succeeded', 'aborted', 'preempted'],
                                              input_keys=['table', 'pose'],
                                              output_keys=['outcome', 'message'])
        with self:
            self.userdata['behavior'] = 'out_to_free_space'
            DoOnExitContainer.add('CLEAR_WAY', ClearTableWay(),
                                  transitions={'succeeded': 'POSE_AS_PATH'})
            DoOnExitContainer.add('POSE_AS_PATH', PoseAsPath(),
                                  transitions={'succeeded': 'AWAY_FROM_TABLE'})
            DoOnExitContainer.add('AWAY_FROM_TABLE', ExePath(),
                                  transitions={'succeeded': 'succeeded',
                                               'aborted': 'TO_FREE_SPACE',
                                               'preempted': 'preempted'})
            DoOnExitContainer.add('TO_FREE_SPACE', Recovery(),
                                  transitions={'succeeded': 'CLEAR_WAY',  # retry
                                               'aborted': 'aborted',
                                               'preempted': 'preempted'})
            DoOnExitContainer.add_finally('RESTORE_WAY', RestoreTableWay())


class FollowWaypoints(DoOnExitContainer):
    """
    Follow a list of waypoints after converting them into a smooth path executable by MBF controllers
    """

    def __init__(self, controller=None):
        super(FollowWaypoints, self).__init__(outcomes=['succeeded', 'aborted', 'preempted'],
                                              input_keys=['waypoints'],
                                              output_keys=['waypoints', 'reached_wp', 'outcome', 'message'])
        with self:
            smach.StateMachine.add('FOLLOW_WP_CTRL', SetNamedConfig('waypoints_following'),
                                   transitions={'succeeded': 'SMOOTH_PATH',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('SMOOTH_PATH', SmoothPath(),
                                   transitions={'succeeded': 'EXE_PATH',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})
            smach.StateMachine.add('EXE_PATH', ExePath(controller,  # already using loose goal reached criteria
                                                       track_progress=True),  # required to retake an interrupted path
                                   transitions={'succeeded': 'succeeded',
                                                'aborted': 'FAILURE',
                                                'preempted': 'preempted'})
            smach.StateMachine.add('FAILURE', ExePathFailed(rospy.get_param('~follow_recovery')),
                                   transitions={'recover': 'RECOVER',
                                                'next_wp': 'NEXT_WP',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('RECOVER', Recovery(),
                                   transitions={'succeeded': 'EXE_PATH',
                                                'aborted': 'FAILURE',
                                                'preempted': 'preempted'},
                                   remapping={'behavior': 'recovery_behavior'})
            smach.StateMachine.add('NEXT_WP', GoToPose(),  # already using loose goal reached criteria
                                   transitions={'succeeded': 'SMOOTH_PATH',
                                                'aborted': 'SKIP_WP',  # also if failed; at least we have skip a wp
                                                'preempted': 'preempted'},
                                   remapping={'target_pose': 'next_wp'})
            self.userdata.ONE = 1
            smach.StateMachine.add('SKIP_WP', UDListSlicing(),
                                   remapping={'list': 'waypoints',
                                              'start': 'ONE'},
                                   transitions={'succeeded': 'EXE_PATH'})
            DoOnExitContainer.add_finally('STANDARD_CTRL', DismissNamedConfig('waypoints_following'))


class TraversePoses(smach.Iterator):
    """ Visit a list of stamped poses """

    def __init__(self):
        super(TraversePoses, self).__init__(outcomes=['succeeded', 'preempted', 'aborted'],
                                            input_keys=['poses'],
                                            output_keys=['outcome', 'message'],
                                            it=lambda: self.userdata.poses,
                                            it_label='target_pose',
                                            exhausted_outcome='succeeded')

        with self:
            smach.Iterator.set_contained_state('GO_TO_POSE',
                                               GoToPose(dist_tolerance=rospy.get_param('~loose_dist_tolerance'),
                                                        angle_tolerance=rospy.get_param('~loose_angle_tolerance')),
                                               loop_outcomes=['succeeded'])


class FollowPose(smach_ros.SimpleActionState):
    def __init__(self, distance=0.0):
        super(FollowPose, self).__init__('pose_follower/follow',
                                         thorp_msgs.FollowPoseAction,
                                         goal_cb=self.make_goal)
        self.follow_distance = distance

    def make_goal(self, ud, goal):
        goal.time_limit = rospy.Duration(25)  # TODO
        goal.distance = self.follow_distance
        goal.stop_at_distance = False  # TODO

    def result_cb(self, ud, status, result):
        pass


class SmoothPath(smach_ros.ServiceState):
    """
    Call path smoother with a list of waypoints to obtain a smooth path connecting them
    """

    def __init__(self):
        super(SmoothPath, self).__init__('waypoints_path/connect_waypoints', thorp_srvs.ConnectWaypoints,
                                         request_cb=self.request_cb,
                                         request_slots=['waypoints'],
                                         response_slots=['path'])

    def request_cb(self, ud, request):
        request.max_steps = 5
        request.max_radius = 0.5
        request.resolution = 0.15
        request.visualize_path = True
