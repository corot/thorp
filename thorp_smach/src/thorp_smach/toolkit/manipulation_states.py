import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import control_msgs.msg as control_msgs

from thorp_toolkit.planning_scene import PlanningScene
from thorp_toolkit.geometry import create_2d_pose, create_3d_pose, pose2d2str
from thorp_toolkit.visualization import Visualization

import config as cfg


class ClearOctomap(smach_ros.ServiceState):
    def __init__(self):
        super(ClearOctomap, self).__init__('clear_octomap', std_srvs.Empty)


class ClearGripper(smach_ros.ServiceState):
    def __init__(self):
        super(ClearGripper, self).__init__('clear_gripper', std_srvs.Empty)


class GripperBusy(smach_ros.ServiceState):
    def __init__(self):
        super(GripperBusy, self).__init__('gripper_busy', std_srvs.Trigger,
                                          response_cb=self.response_cb,
                                          output_keys=['attached_object'],
                                          outcomes=['true', 'false', 'error'])
        self.gripper_busy = False
        self.attached_obj = None

    def response_cb(self, ud, response):
        self.gripper_busy = response.success
        ud['attached_object'] = response.message

    def execute(self, ud):
        outcome = super(GripperBusy, self).execute(ud)
        if outcome == 'succeeded':
            return str(self.gripper_busy).lower()
        return 'error'


class FoldArm(smach.Concurrence):
    """ Concurrently fold arm and close the gripper """
    def __init__(self):
        super(FoldArm, self).__init__(outcomes=['succeeded', 'preempted', 'aborted'],
                                      default_outcome='succeeded',
                                      outcome_map={'succeeded': {'CLOSE_GRIPPER': 'succeeded',
                                                                 'GOTO_RESTING': 'succeeded'},
                                                   'preempted': {'CLOSE_GRIPPER': 'preempted',
                                                                 'GOTO_RESTING': 'preempted'},
                                                   'aborted': {'CLOSE_GRIPPER': 'aborted',
                                                               'GOTO_RESTING': 'aborted'}})
        with self:
            smach.Concurrence.add('CLOSE_GRIPPER',
                                  smach_ros.SimpleActionState('gripper_controller/gripper_action',
                                                              control_msgs.GripperCommandAction,
                                                              goal=control_msgs.GripperCommandGoal(
                                                                  control_msgs.GripperCommand(0.025, 0.0))))
            smach.Concurrence.add('GOTO_RESTING',
                                  StoredConfig('resting'))


class StoredConfig(smach_ros.SimpleActionState):
    """ Move arm into one of the stored configuration (resting, right_up, etc.) """
    def __init__(self, config):
        super(StoredConfig, self).__init__('move_to_target',
                                           thorp_msgs.MoveToTargetAction,
                                           goal=thorp_msgs.MoveToTargetGoal(
                                               thorp_msgs.MoveToTargetGoal.NAMED_TARGET,
                                               config, None, None))


class PickupObject(smach.Iterator):
    """
    Pickup a given object, optionally retrying up to a given number of times.
    If we already have an object attached, we clear the gripper before picking.
    If pickup succeeds, we still check if we have the object physically grasped.
    """
    def __init__(self, attempts=2):
        super(PickupObject, self).__init__(outcomes=['succeeded', 'preempted', 'aborted'],
                                           input_keys=['object_name', 'support_surf', 'max_effort'],
                                           output_keys=[],
                                           it=lambda: range(0, attempts),
                                           it_label='attempt',
                                           exhausted_outcome='aborted')
        with self:
            sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                    input_keys=['object_name', 'support_surf', 'max_effort'],
                                    output_keys=[])
            with sm:
                smach.StateMachine.add('GRIPPER_BUSY?', GripperBusy(),
                                       transitions={'true': 'CLEAR_GRIPPER',
                                                    'false': 'PICKUP_OBJECT',
                                                    'error': 'aborted'})
                smach.StateMachine.add('CLEAR_GRIPPER', ClearGripper(),
                                       transitions={'succeeded': 'PICKUP_OBJECT'})
                smach.StateMachine.add('PICKUP_OBJECT',
                                       smach_ros.SimpleActionState('pickup_object',
                                                                   thorp_msgs.PickupObjectAction,
                                                                   goal_slots=['object_name',
                                                                               'support_surf',
                                                                               'max_effort'],
                                                                   result_slots=['error']),
                                       transitions={'succeeded': 'OBJECT_GRASPED?',
                                                    'preempted': 'preempted',
                                                    'aborted': 'CLEAR_OCTOMAP'})
                smach.StateMachine.add('OBJECT_GRASPED?', GripperBusy(),
                                       transitions={'true': 'succeeded',
                                                    'false': 'CLEAR_ATTACHED',
                                                    'error': 'aborted'})
                smach.StateMachine.add('CLEAR_ATTACHED', ClearGripper(),      # clear the falsely attached object; don't
                                       transitions={'succeeded': 'aborted'})  # retry, as it's not available anymore
                smach.StateMachine.add('CLEAR_OCTOMAP', ClearOctomap(),
                                       transitions={'succeeded': 'continue',
                                                    'preempted': 'preempted',
                                                    'aborted': 'aborted'})
            # TODO: check error and, if collision between parts of the arm, move a bit the arm
            # TODO: check error and, if collision between the arm and an obj, fold arm and redetect
            #    moveit always return -1 -->  not enough info
            smach.Iterator.set_contained_state('', sm, loop_outcomes=['continue'])


class PlaceObject(smach.Iterator):
    """  Place a given object, optionally retrying up to a given number of times  """
    def __init__(self, attempts=2):
        super(PlaceObject, self).__init__(outcomes=['succeeded', 'preempted', 'aborted'],
                                          input_keys=['object_name', 'support_surf', 'place_pose'],
                                          output_keys=[],
                                          it=lambda: range(0, attempts),
                                          it_label='attempt',
                                          exhausted_outcome='aborted')
        with self:
            sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                    input_keys=['object_name', 'support_surf', 'place_pose'],
                                    output_keys=[])
            with sm:
                smach.StateMachine.add('PLACE_OBJECT',
                                       smach_ros.SimpleActionState('place_object',
                                                                   thorp_msgs.PlaceObjectAction,
                                                                   goal_slots=['object_name',
                                                                               'support_surf',
                                                                               'place_pose'],
                                                                   result_slots=[]),
                                       transitions={'succeeded': 'succeeded',
                                                    'preempted': 'preempted',
                                                    'aborted': 'CLEAR_OCTOMAP'})
                smach.StateMachine.add('CLEAR_OCTOMAP', ClearOctomap(),
                                       transitions={'succeeded': 'continue',
                                                    'preempted': 'preempted',
                                                    'aborted': 'aborted'})

            # TODOs:
            #  - check error and, if collision between parts of the arm, move a bit the arm  -->  not enough info
            #  - this doesn't make too much sense as a loop... better try all our tricks and exit
            smach.Iterator.set_contained_state('', sm, loop_outcomes=['continue'])


class PlaceInTray(smach.Sequence):
    """  Place a given object on the tray  """
    def __init__(self):
        super(PlaceInTray, self).__init__(outcomes=['succeeded', 'aborted', 'preempted', 'tray_full'],
                                          connector_outcome='succeeded',
                                          input_keys=['object_name'])
        self.userdata.support_surf = 'tray'
        with self:
            smach.Sequence.add('POSE_IN_TRAY', GetPoseInTray())
            smach.Sequence.add('PLACE_ON_TRAY', PlaceObject(),
                               remapping={'place_pose': 'pose_in_tray'})
            smach.Sequence.add('READJUST_POSE', DisplaceObject(),
                               remapping={'new_pose': 'pose_in_tray'})


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
