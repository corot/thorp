import rospy
import smach
import smach_ros
import dynamic_reconfigure.client

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


class PoseAsPath(smach.State):
    """ Add a path to the userdata containing just the input pose """

    def __init__(self):
        super(PoseAsPath, self).__init__(outcomes=['succeeded'],
                                         input_keys=['pose'],
                                         output_keys=['path'])

    def execute(self, ud):
        ud['path'] = nav_msgs.Path(ud['pose'].header, [ud['pose']])
        return 'succeeded'


class PosesAsPath(smach.State):
    """ Add a path to the userdata containing the list of input poses """

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
        self.prev_dist_tolerance = None
        self.prev_angle_tolerance = None

        # Create a dynamic reconfigure client to change goal tolerance values, if needed
        self.params_ns = '/move_base_flex/' + controller
        try:
            rospy.loginfo("Waiting for '%s' dynamic reconfigure service...", self.params_ns)
            self.reconfigure_client = dynamic_reconfigure.client.Client(self.params_ns, timeout=60)
            rospy.loginfo("Dynamic reconfigure service '%s' alive!", self.params_ns)
        except rospy.exceptions.ROSException as err:
            rospy.logerr("'%s' dynamic reconfiguration not available: %s", self.params_ns, str(err))

    def make_goal(self, ud, goal):
        if self.planner:
            goal.planner = self.planner
        if self.controller:
            goal.controller = self.controller
        if self.rec_behaviors:
            goal.recovery_behaviors = self.rec_behaviors
        if self.dist_tolerance and self.angle_tolerance:
            # Set configured tolerance values and keep previous to restore before leaving the state
            self.prev_dist_tolerance = rospy.get_param(self.params_ns + '/xy_goal_tolerance')
            self.prev_angle_tolerance = rospy.get_param(self.params_ns + '/yaw_goal_tolerance')
            new_config = {'xy_goal_tolerance': self.dist_tolerance,
                          'yaw_goal_tolerance': self.angle_tolerance}
            self.reconfigure_client.update_configuration(new_config)

    def result_cb(self, ud, status, result):
        if self.prev_dist_tolerance and self.prev_angle_tolerance:
            # Restore previous tolerance values
            prev_config = {'xy_goal_tolerance': self.prev_angle_tolerance,
                           'yaw_goal_tolerance': self.prev_angle_tolerance}
            self.reconfigure_client.update_configuration(prev_config)
            self.prev_dist_tolerance = None
            self.prev_angle_tolerance = None


class ExePath(smach_ros.SimpleActionState):
    def __init__(self, controller=cfg.DEFAULT_CONTROLLER, dist_tolerance=None, angle_tolerance=None):
        super(ExePath, self).__init__('move_base_flex/exe_path',
                                      mbf_msgs.ExePathAction,
                                      goal_cb=self.make_goal,
                                      input_keys=['path'],
                                      result_cb=self.result_cb,
                                      result_slots=['outcome', 'message'])
        self.controller = controller
        self.dist_tolerance = dist_tolerance
        self.angle_tolerance = angle_tolerance
        self.prev_dist_tolerance = None
        self.prev_angle_tolerance = None

        # Create a dynamic reconfigure client to change goal tolerance values, if needed
        self.params_ns = '/move_base_flex/' + controller
        try:
            rospy.loginfo("Waiting for '%s' dynamic reconfigure service...", self.params_ns)
            self.reconfigure_client = dynamic_reconfigure.client.Client(self.params_ns, timeout=60)
            rospy.loginfo("Dynamic reconfigure service '%s' alive!", self.params_ns)
        except rospy.exceptions.ROSException as err:
            rospy.logerr("'%s' dynamic reconfiguration not available: %s", self.params_ns, str(err))

    def make_goal(self, ud, goal):
        goal.path = ud['path']
        if self.controller:
            goal.controller = self.controller
        if self.dist_tolerance and self.angle_tolerance:
            # Set configured tolerance values and keep previous to restore before leaving the state
            self.prev_dist_tolerance = rospy.get_param(self.params_ns + '/xy_goal_tolerance')
            self.prev_angle_tolerance = rospy.get_param(self.params_ns + '/yaw_goal_tolerance')
            new_config = {'xy_goal_tolerance': self.dist_tolerance,
                          'yaw_goal_tolerance': self.angle_tolerance}
            self.reconfigure_client.update_configuration(new_config)

    def result_cb(self, ud, status, result):
        if self.prev_dist_tolerance and self.prev_angle_tolerance:
            # Restore previous tolerance values
            prev_config = {'xy_goal_tolerance': self.prev_angle_tolerance,
                           'yaw_goal_tolerance': self.prev_angle_tolerance}
            self.reconfigure_client.update_configuration(prev_config)
            self.prev_dist_tolerance = None
            self.prev_angle_tolerance = None

    def _goal_feedback_cb(self, feedback):
        super(ExePath, self)._goal_feedback_cb(feedback)


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
            ud['path'].poses = ud['path'].poses[current_waypoint:]
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
                next_wp_pose = ud['path'].poses[0]
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
