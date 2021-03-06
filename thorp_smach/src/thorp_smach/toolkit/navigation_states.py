import rospy
import smach
import smach_ros

import geometry_msgs.msg as geo_msgs
import nav_msgs.msg as nav_msgs
import mbf_msgs.msg as mbf_msgs
import rr_path_smoother.srv as ps_srvs
import thorp_msgs.msg as thorp_msgs

from thorp_toolkit.geometry import TF2, pose2d2str, heading, quaternion_msg_from_yaw
from thorp_toolkit.reconfigure import Reconfigure
from thorp_toolkit.semantic_layer import SemanticLayer
from thorp_toolkit.progress_tracker import ProgressTracker

from common_states import SetNamedConfig, DismissNamedConfig

import config as cfg

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


class AddSemanticObj(smach.State):
    """ Add an object to the semantic map of one or both costmaps """

    def __init__(self):
        super(AddSemanticObj, self).__init__(outcomes=['succeeded'],
                                             input_keys=['name', 'type', 'pose', 'size', 'costmap'])

    def execute(self, ud):
        SemanticLayer().add_object(ud['name'], ud['type'], ud['pose'], ud['size'], ud['costmap'])
        return 'succeeded'


class RemoveSemanticObj(smach.State):
    """ Remove an object from the semantic map of one or both costmaps """

    def __init__(self):
        super(RemoveSemanticObj, self).__init__(outcomes=['succeeded'],
                                                input_keys=['name', 'type', 'costmap'])

    def execute(self, ud):
        SemanticLayer().remove_object(ud['name'], ud['type'], ud['costmap'])
        return 'succeeded'


class GoToPose(smach_ros.SimpleActionState):
    def __init__(self,
                 planner=cfg.DEFAULT_PLANNER,
                 controller=cfg.DEFAULT_CONTROLLER,
                 rec_behaviors=cfg.MOVE_BASE_RECOVERY,
                 dist_tolerance=None, angle_tolerance=None):
        super(GoToPose, self).__init__('move_base_flex/move_base',
                                       mbf_msgs.MoveBaseAction,
                                       goal_cb=self.make_goal,
                                       goal_slots=['target_pose'],
                                       result_cb=self.result_cb,
                                       result_slots=['outcome', 'message'])
        self.planner = planner
        self.controller = controller
        self.rec_behaviors = rec_behaviors
        self.dist_tolerance = dist_tolerance
        self.angle_tolerance = angle_tolerance
        self.params_ns = '/move_base_flex/' + controller

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
    def __init__(self, controller=cfg.DEFAULT_CONTROLLER, dist_tolerance=None, angle_tolerance=None,
                 track_progress=False):
        super(ExePath, self).__init__('move_base_flex/exe_path',
                                      mbf_msgs.ExePathAction,
                                      goal_cb=self.make_goal,
                                      result_cb=self.result_cb,
                                      result_slots=['outcome', 'message'],
                                      input_keys=['path', 'waypoints'],
                                      output_keys=['reached_wp'])
        self.controller = controller
        self.dist_tolerance = dist_tolerance
        self.angle_tolerance = angle_tolerance
        self.track_progress = track_progress
        self.params_ns = '/move_base_flex/' + controller

        # Show target pose (MBF only shows it when calling get_path)
        self.target_pose_pub = rospy.Publisher('/move_base_flex/current_goal', geo_msgs.PoseStamped, queue_size=1)

    def make_goal(self, ud, goal):
        goal.path = ud['path']
        if self.controller:
            goal.controller = self.controller
        if self.dist_tolerance and self.angle_tolerance:
            # Set configured tolerance values
            Reconfigure().update_config(self.params_ns, {'xy_goal_tolerance': self.dist_tolerance,
                                                         'yaw_goal_tolerance': self.angle_tolerance})
        if goal.path.poses:
            self.target_pose_pub.publish(goal.path.poses[-1])

        if self.track_progress:
            self.progress_tracker = ProgressTracker(ud['waypoints'], cfg.WP_REACHED_THRESHOLD)

    def result_cb(self, ud, status, result):
        if self.dist_tolerance and self.angle_tolerance:
            # Restore previous tolerance values before leaving the state
            Reconfigure().restore_config(self.params_ns, ['xy_goal_tolerance', 'yaw_goal_tolerance'])
        if self.track_progress:
            ud['reached_wp'] = self.progress_tracker.reached_waypoint()
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
        goal.controller = cfg.DEFAULT_CONTROLLER


class Recovery(smach_ros.SimpleActionState):
    def __init__(self):
        super(Recovery, self).__init__('move_base_flex/recovery',
                                       mbf_msgs.RecoveryAction,
                                       goal_slots=['behavior'],
                                       result_slots=['outcome', 'message'])


class ExePathFailed(smach.State):
    """ Handle failures on ExePath state """

    def __init__(self):
        super(ExePathFailed, self).__init__(outcomes=['recover', 'next_wp', 'aborted'],
                                            input_keys=['path', 'waypoints', 'reached_wp', 'outcome', 'message'],
                                            output_keys=['path', 'waypoints', 'next_wp', 'recovery_behavior'])
        self.recovery_behaviors = cfg.EXE_PATH_RECOVERY
        self.consecutive_failures = 0
        self.next_waypoint = None

    def execute(self, ud):
        # Cut path up to current waypoint, so we don't redo it from the beginning after recovering
        # ExePath must provide this specific message at the end of the result message in case of failure
        cwp_index = ud['message'].find('current waypoint: ')
        if cwp_index >= 0:
            # TODO I must adapt this to integrate the progress tracker with the smoothed path logic (or remove entirely path follower support)
            self.next_waypoint = int(ud['message'][cwp_index + len('current waypoint: '):])
            ud['path'].poses = ud['path'].poses[self.next_waypoint:]
        elif ud['reached_wp'] is not None and ud['reached_wp'] != self.next_waypoint:
            # TEB logic using progress tracker:   is very brittle, but seems to work!  hope I can do better w/ BTs
            #  ignore None on 'reached_wp', as it's normally a failure after a recovery attempt
            self.next_waypoint = ud['reached_wp'] + 1
            ud['waypoints'] = ud['waypoints'][self.next_waypoint:]
        # else:
        #     rospy.logwarn("No current waypoint provided")
        #     self.next_waypoint = -1

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
            if self.next_waypoint is not None and ud['waypoints']:
                next_wp_pose = ud['waypoints'][0]
                rospy.loginfo("Navigate to the next waypoint: %d, %s", self.next_waypoint, pose2d2str(next_wp_pose))
                ud['next_wp'] = next_wp_pose
                if len(ud['waypoints']) > 1:
                    # if not the last, consume a waypoint, so we try with the 2nd next if we fail to go to the next
                    ud['waypoints'].pop(0)
                return 'next_wp'

            return 'aborted'


class ClearTableWay(smach.State):
    """
    Clear an area on local costmap so the controller can approach the table, and restore it after a given time
    """

    def __init__(self):
        super(ClearTableWay, self).__init__(outcomes=['succeeded'],
                                            input_keys=['table', 'pose'])

    def execute(self, ud):
        SemanticLayer().add_object(ud['table'].name, 'free_space', ud['pose'], [0.5, 0.4], 'local')
        rospy.Timer(rospy.Duration(cfg.CLEAR_TABLE_WAY_TIMEOUT),
                    lambda te: SemanticLayer().remove_object(ud['table'].name, 'free_space', 'local'),
                    oneshot=True)
        return 'succeeded'


class AlignToTable(smach.Sequence):
    def __init__(self):
        super(AlignToTable, self).__init__(outcomes=['succeeded',
                                                     'aborted',
                                                     'preempted'],
                                           connector_outcome='succeeded',
                                           input_keys=['table', 'pose'],
                                           output_keys=['outcome', 'message'])
        with self:
            smach.Sequence.add('CLEAR_WAY', ClearTableWay())
            smach.Sequence.add('POSE_AS_PATH', PoseAsPath())
            smach.Sequence.add('PRECISE_CTRL', SetNamedConfig('precise_controlling'))
            smach.Sequence.add('ALIGN_TO_TABLE', ExePath())


class DetachFromTable(smach.Sequence):
    def __init__(self):
        super(DetachFromTable, self).__init__(outcomes=['succeeded',
                                                        'aborted',
                                                        'preempted'],
                                              connector_outcome='succeeded',
                                              input_keys=['table', 'pose'],
                                              output_keys=['outcome', 'message'])
        with self:
            smach.Sequence.add('POSE_AS_PATH', PoseAsPath())
            smach.Sequence.add('AWAY_FROM_TABLE', ExePath())
            smach.Sequence.add('STANDARD_CTRL', DismissNamedConfig('precise_controlling'))


class ExeSparsePath(smach.StateMachine):
    def __init__(self):
        super(ExeSparsePath, self).__init__(outcomes=['succeeded',
                                                      'aborted',
                                                      'preempted'],
                                            input_keys=['path'],
                                            output_keys=['outcome', 'message'])
        with self:
            smach.StateMachine.add('EXE_PATH', ExePath(cfg.FOLLOW_CONTROLLER,
                                                       cfg.LOOSE_DIST_TOLERANCE,
                                                       cfg.INF_ANGLE_TOLERANCE),
                                   transitions={'succeeded': 'succeeded',
                                                'aborted': 'FAILURE',
                                                'preempted': 'preempted'})
            smach.StateMachine.add('FAILURE', ExePathFailed(),
                                   transitions={'recover': 'RECOVER',
                                                'next_wp': 'NEXT_WP',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('RECOVER', Recovery(),
                                   transitions={'succeeded': 'EXE_PATH',
                                                'aborted': 'FAILURE',
                                                'preempted': 'preempted'},
                                   remapping={'behavior': 'recovery_behavior'})
            smach.StateMachine.add('NEXT_WP', GoToPose(dist_tolerance=cfg.LOOSE_DIST_TOLERANCE,
                                                       angle_tolerance=cfg.INF_ANGLE_TOLERANCE),
                                   transitions={'succeeded': 'EXE_PATH',
                                                'aborted': 'EXE_PATH',  # also if failed; at least we have skip a wp
                                                'preempted': 'preempted'},
                                   remapping={'target_pose': 'next_wp'})


class FollowWaypoints(smach.DoOnExit):
    """
    Follow a list of waypoints after converting them into a smooth path executable by an MBF controller
    """
    def __init__(self, controller=cfg.DEFAULT_CONTROLLER):
        super(FollowWaypoints, self).__init__(outcomes=['succeeded',
                                                        'aborted',
                                                        'preempted'],
                                              input_keys=['waypoints'],
                                              output_keys=['outcome', 'message'])
        with self:
            smach.StateMachine.add('FOLLOW_WP_CTRL', SetNamedConfig('waypoints_following'),
                                   transitions={'succeeded': 'SMOOTH_PATH',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('SMOOTH_PATH', SmoothPath(),
                                   transitions={'succeeded': 'EXE_PATH',
                                                'aborted': 'FAILURE',
                                                'preempted': 'preempted'})
            smach.StateMachine.add('EXE_PATH', ExePath(controller,
                                                       cfg.LOOSE_DIST_TOLERANCE,
                                                       cfg.INF_ANGLE_TOLERANCE,  # ignore final yaw
                                                       True),                    # track progress
                                   transitions={'succeeded': 'succeeded',
                                                'aborted': 'FAILURE',
                                                'preempted': 'preempted'})
            smach.StateMachine.add('FAILURE', ExePathFailed(),
                                   transitions={'recover': 'RECOVER',
                                                'next_wp': 'NEXT_WP',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('RECOVER', Recovery(),
                                   transitions={'succeeded': 'EXE_PATH',
                                                'aborted': 'FAILURE',
                                                'preempted': 'preempted'},
                                   remapping={'behavior': 'recovery_behavior'})
            smach.StateMachine.add('NEXT_WP', GoToPose(dist_tolerance=cfg.LOOSE_DIST_TOLERANCE,
                                                       angle_tolerance=cfg.INF_ANGLE_TOLERANCE),
                                   transitions={'succeeded': 'EXE_PATH',
                                                'aborted': 'EXE_PATH',  # also if failed; at least we have skip a wp
                                                'preempted': 'preempted'},
                                   remapping={'target_pose': 'next_wp'})
            smach.DoOnExit.add_finally('STANDARD_CTRL', DismissNamedConfig('waypoints_following'))


class TraversePoses(smach.Iterator):
    """ Visit a list of stamped poses """
    def __init__(self, dist_tolerance=cfg.LOOSE_DIST_TOLERANCE, angle_tolerance=cfg.LOOSE_ANGLE_TOLERANCE):
        super(TraversePoses, self).__init__(outcomes=['succeeded', 'preempted', 'aborted'],
                                            input_keys=['poses'],
                                            output_keys=['outcome', 'message'],
                                            it=lambda: self.userdata.poses,
                                            it_label='target_pose',
                                            exhausted_outcome='succeeded')

        with self:
            smach.Iterator.set_contained_state('GO_TO_POSE', GoToPose(dist_tolerance=dist_tolerance,
                                                                      angle_tolerance=angle_tolerance),
                                               loop_outcomes=['succeeded'])


class FollowPose(smach_ros.SimpleActionState):
    def __init__(self, distance=0.0):
        super(FollowPose, self).__init__('pose_follower/follow',
                                         thorp_msgs.FollowPoseAction,
                                         goal_cb=self.make_goal)
        self.follow_distance = distance

    def make_goal(self, ud, goal):
        goal.time_limit = rospy.Duration(25)   # TODO
        goal.distance = self.follow_distance
        goal.stop_at_distance = False  # TODO

    def result_cb(self, ud, status, result):
        pass


class SmoothPath(smach_ros.ServiceState):
    """
    Call path smoother with a list of waypoints to obtain a collision-free, smooth path connecting them
    """
    def __init__(self):
        super(SmoothPath, self).__init__('path_smoother/connect_waypoints', ps_srvs.ConnectWaypoints,
                                         request_cb=self.request_cb,
                                         request_slots=['waypoints'],
                                         response_slots=['path'])

    def request_cb(self, ud, request):
        request.max_steps = 5
        request.max_radius = 0.5
        request.resolution = 0.15
        request.publish_output = True
