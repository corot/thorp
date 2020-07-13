import rospy
import smach
import smach_ros

import nav_msgs.msg as nav_msgs
import mbf_msgs.msg as mbf_msgs

from thorp_toolkit.geometry import TF2

import config as cfg


class GetRobotPose(smach.State):
    """ Add current robot pose to userdata """

    def __init__(self):
        super(GetRobotPose, self).__init__(outcomes=['succeeded', 'aborted'],
                                           output_keys=['robot_pose'])

    def execute(self, ud):
        try:
            # set as Pose, no PoseStamped, as required by FindRoomSequenceWithCheckpointsGoal
            ud['robot_pose'] = TF2().transform_pose(None, 'base_footprint', 'map').pose
            return 'succeeded'
        except rospy.ROSException as err:
            rospy.logerr("Get robot pose failed: %s", str(err))
            return 'aborted'


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
                                       result_slots=['outcome', 'message'])
        self.planner = planner
        self.controller = controller
        self.rec_behaviors = rec_behaviors
        self.dist_tolerance = dist_tolerance
        self.angle_tolerance = angle_tolerance

    def make_goal(self, ud, goal):
        if self.planner:
            goal.planner = self.planner
        if self.controller:
            goal.controller = self.controller
        if self.rec_behaviors:
            goal.recovery_behaviors = self.rec_behaviors
        if self.dist_tolerance and self.angle_tolerance:
            goal.dist_tolerance = self.dist_tolerance
            goal.angle_tolerance = self.angle_tolerance
            goal.tolerance_from_action = True

class GoToPose___NO_ARGS_ALL_USERDATA(smach_ros.SimpleActionState):  # TODO decide and remove one
    def __init__(self):
        super(GoToPose, self).__init__('move_base_flex/move_base',
                                       mbf_msgs.MoveBaseAction,
                                       goal_cb=self.make_goal,
                                       goal_slots=['target_pose'],
                                       input_keys=['planner', 'controller', 'dist_tolerance', 'angle_tolerance'],
                                       result_slots=['outcome', 'message'])

    def make_goal(self, ud, goal):
        if 'planner' in ud:
            goal.planner = ud['planner']
        if 'controller' in ud:
            goal.controller = ud['controller']
        if 'dist_tolerance' in ud and 'angle_tolerance' in ud:
            goal.dist_tolerance = ud['dist_tolerance']
            goal.angle_tolerance = ud['angle_tolerance']
            goal.tolerance_from_action = True


class ExePath(smach_ros.SimpleActionState):
    def __init__(self, controller=cfg.DEFAULT_CONTROLLER, dist_tolerance=None, angle_tolerance=None):
        super(ExePath, self).__init__('move_base_flex/exe_path',
                                      mbf_msgs.ExePathAction,
                                      goal_cb=self.make_goal,
                                      input_keys=['path'],
                                      result_slots=['outcome', 'message'])
        self.controller = controller
        self.dist_tolerance = dist_tolerance
        self.angle_tolerance = angle_tolerance

    def make_goal(self, ud, goal):
        goal.path = nav_msgs.Path(ud['path'][0].header, ud['path'])
        if self.controller:
            goal.controller = self.controller
        if self.dist_tolerance and self.angle_tolerance:
            goal.dist_tolerance = self.dist_tolerance
            goal.angle_tolerance = self.angle_tolerance
            goal.tolerance_from_action = True

    def _goal_feedback_cb(self, feedback):
        super(ExePath, self)._goal_feedback_cb(feedback)


class ExePath___NO_ARGS_ALL_USERDATA(smach_ros.SimpleActionState):  # TODO decide and remove one
    def __init__(self):
        super(ExePath, self).__init__('move_base_flex/exe_path',
                                      mbf_msgs.ExePathAction,
                                      goal_cb=self.make_goal,
                                      input_keys=['path', 'controller', 'dist_tolerance', 'angle_tolerance'],
                                      result_slots=['outcome', 'message'])

    def make_goal(self, ud, goal):
        goal.path = nav_msgs.Path(ud['path'][0].header, ud['path'])
        if 'controller' in ud:
            goal.controller = ud['controller']
        if 'dist_tolerance' in ud and 'angle_tolerance' in ud:
            goal.dist_tolerance = ud['dist_tolerance']
            goal.angle_tolerance = ud['angle_tolerance']
            goal.tolerance_from_action = True


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
                                            input_keys=['path', 'outcome', 'message'],
                                            output_keys=['path', 'next_wp', 'recovery_behavior'])
        self.recovery_behaviors = cfg.EXE_PATH_RECOVERY
        self.consecutive_failures = 0

    def execute(self, ud):
        # Cut path up to current waypoint, so we don't redo it from the beginning after recovering
        # ExePath must provide this specific message at the end of the result message in case of failure
        cwp_index = ud['message'].find('current waypoint: ')
        if cwp_index >= 0:
            current_waypoint = int(ud['message'][cwp_index + len('current waypoint: '):])
            ud['path'] = ud['path'][current_waypoint:]
        else:
            rospy.logwarn("No current waypoint provided")
            current_waypoint = -1

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
            if current_waypoint >= 0:
                next_wp_pose = ud['path'][0]
                rospy.loginfo("Try to navigate to the next waypoint: %d, %s", current_waypoint, next_wp_pose)
                ud['next_wp'] = next_wp_pose
                return 'next_wp'

            return 'aborted'


class ExeSparsePath(smach.StateMachine):
    def __init__(self):
        super(ExeSparsePath, self).__init__(outcomes=['succeeded',
                                                      'aborted',
                                                      'preempted'],
                                            input_keys=['path'],
                                            output_keys=['outcome', 'message'])
        with self:
            smach.StateMachine.add('EXE_PATH', ExePath(cfg.TRAVERSE_CONTROLLER,
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
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'},
                                   remapping={'behavior': 'recovery_behavior'})
            smach.StateMachine.add('NEXT_WP', GoToPose(dist_tolerance=cfg.LOOSE_DIST_TOLERANCE,
                                                       angle_tolerance=cfg.INF_ANGLE_TOLERANCE),
                                   transitions={'succeeded': 'EXE_PATH',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'},
                                   remapping={'target_pose': 'next_wp'})


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
