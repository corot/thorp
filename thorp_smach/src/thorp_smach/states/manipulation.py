import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import thorp_msgs.srv as thorp_srvs
import control_msgs.msg as control_msgs

from thorp_toolkit.planning_scene import PlanningScene
from thorp_toolkit.geometry import pose2d2str, create_3d_pose

from .userdata import UDExtractAttr


class ClearOctomap(smach_ros.ServiceState):
    def __init__(self):
        super(ClearOctomap, self).__init__('clear_octomap', std_srvs.Empty)


class ClearGripper(smach_ros.ServiceState):
    def __init__(self):
        super(ClearGripper, self).__init__('clear_gripper', std_srvs.Empty)


class ObjAttached(smach_ros.ServiceState):
    """
    Check if we have a collision object attached to the gripper, according to the planning scene
    """

    def __init__(self):
        super(ObjAttached, self).__init__('obj_attached', std_srvs.Trigger,
                                          response_cb=self.response_cb,
                                          output_keys=['attached_object'],
                                          outcomes=['true', 'false', 'error'])
        self.obj_attached = False
        self.attached_obj = None

    def response_cb(self, ud, response):
        self.obj_attached = response.success
        ud['attached_object'] = response.message

    def execute(self, ud):
        outcome = super(ObjAttached, self).execute(ud)
        if outcome == 'succeeded':
            return str(self.obj_attached).lower()
        return 'error'


class GripperBusy(smach_ros.ServiceState):
    """
    Check if the gripper is physically holding an object, regardless of what the planning scene says
    """

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
        super(StoredConfig, self).__init__('manipulation/move_to_target',
                                           thorp_msgs.MoveToTargetAction,
                                           goal=thorp_msgs.MoveToTargetGoal(
                                               thorp_msgs.MoveToTargetGoal.NAMED_TARGET,
                                               config, None, None))


class PickupObject(smach.Iterator):
    """
    Pickup a given object, optionally retrying up to a given number of times.
    If we already have an object attached, we clear the gripper before pickup.
    If pickup succeeds, we still check if we have the object physically grasped.
    """

    def __init__(self, attempts=2):
        super(PickupObject, self).__init__(outcomes=['succeeded', 'preempted', 'aborted'],
                                           input_keys=['object', 'surface', 'max_effort', 'tightening'],
                                           output_keys=[],
                                           it=lambda: range(0, attempts),
                                           it_label='attempt',
                                           exhausted_outcome='aborted')
        with self:
            sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                    input_keys=['object', 'surface', 'max_effort', 'tightening'],
                                    output_keys=[])
            with sm:
                smach.StateMachine.add('GET_OBJ_NAME', UDExtractAttr('id', 'object', 'object_name'),
                                       transitions={'succeeded': 'GET_SURF_NAME'})
                smach.StateMachine.add('GET_SURF_NAME', UDExtractAttr('id', 'surface', 'support_surf'),
                                       transitions={'succeeded': 'OBJ_ATTACHED?'})
                smach.StateMachine.add('OBJ_ATTACHED?', ObjAttached(),
                                       transitions={'true': 'CLEAR_GRIPPER',
                                                    'false': 'GRIPPER_BUSY?',
                                                    'error': 'aborted'})
                smach.StateMachine.add('GRIPPER_BUSY?', GripperBusy(),
                                       transitions={'true': 'CLEAR_GRIPPER',
                                                    'false': 'PICKUP_OBJECT',
                                                    'error': 'aborted'})
                smach.StateMachine.add('CLEAR_GRIPPER', ClearGripper(),
                                       transitions={'succeeded': 'PICKUP_OBJECT'})
                smach.StateMachine.add('PICKUP_OBJECT',
                                       smach_ros.SimpleActionState('manipulation/pickup_object',
                                                                   thorp_msgs.PickupObjectAction,
                                                                   goal_slots=['object_name',
                                                                               'support_surf',
                                                                               'max_effort',
                                                                               'tightening'],
                                                                   result_slots=['error']),
                                       transitions={'succeeded': 'OBJECT_GRASPED?',
                                                    'preempted': 'preempted',
                                                    'aborted': 'CLEAR_OCTOMAP'})
                smach.StateMachine.add('OBJECT_GRASPED?', GripperBusy(),
                                       transitions={'true': 'succeeded',
                                                    'false': 'CLEAR_ATTACHED',
                                                    'error': 'aborted'})
                smach.StateMachine.add('CLEAR_ATTACHED', ClearGripper(),  # clear the falsely attached object; don't
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
                                          input_keys=['object', 'surface', 'place_pose'],
                                          output_keys=[],
                                          it=lambda: range(0, attempts),
                                          it_label='attempt',
                                          exhausted_outcome='aborted')
        with self:
            sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                    input_keys=['object', 'surface', 'place_pose'],
                                    output_keys=[])
            with sm:
                smach.StateMachine.add('GET_OBJ_NAME', UDExtractAttr('id', 'object', 'object_name'),
                                       transitions={'succeeded': 'GET_SURF_NAME'})
                smach.StateMachine.add('GET_SURF_NAME', UDExtractAttr('id', 'surface', 'support_surf'),
                                       transitions={'succeeded': 'PLACE_OBJECT'})
                smach.StateMachine.add('PLACE_OBJECT',
                                       smach_ros.SimpleActionState('manipulation/place_object',
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


class PlaceOnTray(smach.Sequence):
    """  Place a given object on the tray  """

    def __init__(self):
        super(PlaceOnTray, self).__init__(outcomes=['succeeded', 'aborted', 'preempted', 'tray_full'],
                                          connector_outcome='succeeded',
                                          input_keys=['object'])
        # TODO create a fake surface because PlaceObject expects a CollisionObject (but just to get the id)
        # TODO as on BT C++, I can just use surface name (WARN: goal field is called support_surf)
        PlanningScene().add_tray(create_3d_pose(0, 0, 0, 0, 0, 0, 'tray_link'), (0, 0, 0))
        self.userdata.surface = PlanningScene().get_obj('tray')

        with self:
            smach.Sequence.add('POSE_ON_TRAY', NextPoseOnTray())
            smach.Sequence.add('PLACE_ON_TRAY', PlaceObject(),
                               remapping={'place_pose': 'pose_on_tray'})
            smach.Sequence.add('MOVE_TO_TRAY', AddObjToTray())


class NextPoseOnTray(smach_ros.ServiceState):
    """
    Request the next pose where to put an object on the tray.
    """

    def __init__(self):
        super(NextPoseOnTray, self).__init__('manipulation/tray/get_next_pose', thorp_srvs.TrayNextPose,
                                             response_cb=self.response_cb,
                                             outcomes=['succeeded', 'tray_full'],
                                             output_keys=['pose_on_tray'])

    def response_cb(self, ud, response):
        ud['pose_on_tray'] = response.pose_on_tray

    def execute(self, ud):
        outcome = super(NextPoseOnTray, self).execute(ud)
        if outcome == 'succeeded':
            return outcome
        return 'tray_full'


class RemoveObject(smach.State):
    """
    Remove collision object from the planning scene
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['object_name'])

    def execute(self, ud):
        rospy.loginfo("Removing object '%s' from planning scene", ud['object_name'])
        PlanningScene().remove_obj(ud['object_name'])
        return 'succeeded'


class ClearPlanningScene(smach_ros.ServiceState):
    """
    Clear the planning scene, optionally sparing the tray and its content
    """

    def __init__(self, keep_tray=True):
        super(ClearPlanningScene, self).__init__('manipulation/clear_planning_scene', thorp_srvs.ClearPlanningScene,
                                                 request_cb=self.request_cb)
        self.keep_tray = keep_tray

    def request_cb(self, ud, request):
        request.keep_tray = self.keep_tray


class DisplaceObject(smach.State):
    """
    Displace a collision object in the planning scene
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['object', 'new_pose'])

    def execute(self, ud):
        rospy.loginfo("Object '%s' pose readjusted to %s", ud['object'].id, pose2d2str(ud['new_pose']))
        PlanningScene().displace_obj(ud['object'].id, ud['new_pose'])
        return 'succeeded'


class AddObjToTray(smach_ros.ServiceState):
    """
    Add a collision object to the tray.
    """

    def __init__(self):
        super(AddObjToTray, self).__init__('manipulation/tray/add_object', thorp_srvs.TrayAddObject,
                                           request_cb=self.request_cb,
                                           input_keys=['object', 'pose_on_tray'])

    def request_cb(self, ud, request):
        request.object_name = ud['object'].id
        request.pose_on_tray = ud['pose_on_tray']
