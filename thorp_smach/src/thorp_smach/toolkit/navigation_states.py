import rospy
import smach
import smach_ros

import nav_msgs.msg as nav_msgs
import mbf_msgs.msg as mbf_msgs

from thorp_toolkit.geometry import TF2


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
