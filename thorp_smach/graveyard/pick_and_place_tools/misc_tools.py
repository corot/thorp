#!/usr/bin/env python
import dynamic_reconfigure.client
import tf
from thorp_smach.state_machines.state_machines_imports import *
from thorp_smach.pick_and_place_tools.msg_imports import *


class AttachObjectToRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done'],
                             input_keys=['collision_object',
                                         'attach'],
                             output_keys=[])
        self._pub_collision_object = rospy.Publisher("attached_collision_object",
                                                     moveit_msgs.AttachedCollisionObject,
                                                     latch=True)
    def execute(self, userdata):
        attached_object = moveit_msgs.AttachedCollisionObject()
        attached_object.link_name = "palm_link"
        attached_object.object = userdata.collision_object
        if userdata.attach:
            rospy.loginfo("Preparing to attach object ...")
#            attached_object.object.header.frame_id = attached_object.link_name
#            new_pose = geometry_msgs.Pose()
#            quat = tf.transformations.quaternion_from_euler(0, 0, 0)
#            new_pose.orientation = geometry_msgs.Quaternion(*quat)
#            attached_object.object.mesh_poses[0] = new_pose
            attached_object.object.operation = moveit_msgs.CollisionObject.ADD
        else:
            rospy.loginfo("Preparing to detach object ...")
            attached_object.object.id = "" # remove all objects from link
            attached_object.object.operation = moveit_msgs.CollisionObject.REMOVE
        attached_object.touch_links.append("lower_arm_link");
        attached_object.touch_links.append("wrist_link");
        attached_object.touch_links.append("palm_link");
        attached_object.touch_links.append("gripper_link");
        attached_object.touch_links.append("gripper_center_link");
        attached_object.touch_links.append("gripper_camera_link");
        attached_object.touch_links.append("finger_left_knuckle_1_link");
        attached_object.touch_links.append("finger_left_knuckle_2_link");
        attached_object.touch_links.append("finger_right_knuckle_1_link");
        attached_object.touch_links.append("finger_right_knuckle_2_link");
        self._pub_collision_object.publish(attached_object)

        duration = 2.0
        rospy.loginfo('Waiting for ' + str(duration) + ' seconds.')
        rospy.sleep(duration) # wait a bit to make sure all subscribers will receive the message
        if userdata.attach:
            rospy.loginfo("Attached object.");
        else:
            rospy.loginfo("Detached object.");
        return 'done'

class AddObjectsToPlanningScene(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done'],
                             input_keys=['recognised_objects',
                                         'object_names',
                                         'objects_info',
                                         'error_code',
                                         'error_message'],
                             output_keys=['error_code',
                                          'error_message'])
        self._pub_collision_object = rospy.Publisher("collision_object",
                                                     moveit_msgs.CollisionObject,
                                                     latch=True)
        rospy.loginfo("Collision objects publisher initialised.")

    def execute(self, userdata):
        rospy.loginfo('Publishing recognised objects as collision objects ...')
        for recognised_object in userdata.recognised_objects.objects:
            collision_object = moveit_msgs.CollisionObject()
            collision_object.header = recognised_object.pose.header
            collision_object.header.stamp = rospy.Time.now()
            recognised_object_nr = userdata.recognised_objects.objects.index(recognised_object)
            collision_object.id = userdata.object_names[recognised_object_nr]
            collision_object.type = recognised_object.type
            shape = shape_msgs.SolidPrimitive()
            shape.type = shape_msgs.SolidPrimitive.CYLINDER
            shape.dimensions.append(0.20) # CYLINDER_HEIGHT
            shape.dimensions.append(0.05) # CYLINDER_RADIUS
            collision_object.primitives.append(shape)
            primitive_pose = geometry_msgs.Pose()
            primitive_pose.position.x = recognised_object.pose.pose.pose.position.x
            primitive_pose.position.y = recognised_object.pose.pose.pose.position.y
            primitive_pose.position.z = recognised_object.pose.pose.pose.position.z + 0.05
            primitive_pose.orientation = recognised_object.pose.pose.pose.orientation
            collision_object.primitive_poses.append(primitive_pose)
#            collision_object.meshes.append(userdata.objects_info[recognised_object_nr].information.ground_truth_mesh)
#            collision_object.mesh_poses.append(recognised_object.pose.pose.pose)
            collision_object.operation = moveit_msgs.CollisionObject.ADD
            self._pub_collision_object.publish(collision_object)

        duration = 2.0
        rospy.loginfo('Waiting for ' + str(duration) + ' seconds.')
        rospy.sleep(duration) # wait a bit to make sure all subscribers will receive the message
        rospy.loginfo("Added objects to planning scene.");

        userdata.error_message = "Published recognised objects as collision objects to the planning scene."
        return 'done'

class ClearCollisionObjects(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['cleared',
                                       'clearing_failed'])

        self._pub_collision_objects = rospy.Publisher("collision_object",
                                                      moveit_msgs.CollisionObject,
                                                      latch=True)
        self._pub_attached_collision_objects = rospy.Publisher("attached_collision_object",
                                                               moveit_msgs.AttachedCollisionObject,
                                                               latch=True)

    def execute(self, userdata):
        object = moveit_msgs.CollisionObject()
        rospy.loginfo("Removing all collision objects ...")
        object.header.stamp = rospy.Time.now()
        object.header.frame_id = 'base_footprint'
        object.operation = moveit_msgs.CollisionObject.REMOVE
        self._pub_collision_objects.publish(object)
        rospy.loginfo("Preparing to remove all attached objects ...")
        attached_object = moveit_msgs.AttachedCollisionObject()
        attached_object.link_name = 'palm_link'
        attached_object.object.id = ""
        attached_object.object.operation = moveit_msgs.CollisionObject.REMOVE
        self._pub_attached_collision_objects.publish(attached_object)

        duration = 2.0
        rospy.loginfo('Waiting for ' + str(duration) + ' seconds.')
        rospy.sleep(duration) # wait a bit to make sure all subscribers will receive the message
        rospy.loginfo("Removed all attached objects.");
        return 'cleared'


class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done'],
                             input_keys=['duration'])

    def execute(self, userdata):
        rospy.loginfo('Waiting for ' + str(userdata.duration) + ' seconds.')
        rospy.sleep(userdata.duration)
        return 'done'

class PickChecker(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success',
                                       'error'],
                             input_keys=['desired_gripper_position'])
        self._desired_gripper_position = 0.0
        self._keep_going = True
        self._success = False

    def jointStateCB(self, data):
        if self._keep_going:
            for joint in data.name:
                if joint == "gripper":
                    joint_nr = data.name.index(joint)
                    if (data.position[joint_nr] >= (self._desired_gripper_position + 0.05)):
                        rospy.loginfo("Gripper is at the desired state! (current: " + str(data.position[joint_nr])
                                     + ", desired: " + str(self._desired_gripper_position) + ")")
                        self._success = True
                        self._keep_going = False
                    else:
                        rospy.logerr("Gripper is not in the desired state! (current: " + str(data.position[joint_nr])
                                     + ", desired: " + str(self._desired_gripper_position) + ")")
                        self._success = False
                        self._keep_going = False
                    return
            rospy.logerr("Couldn't find a joint state for 'gripper'")
            self._success = False
            self._keep_going = False
        return

    def execute(self, userdata):
        self._desired_gripper_position = userdata.desired_gripper_position
        self._sub_joint_states = rospy.Subscriber("joint_states", sensor_msgs.JointState, self.jointStateCB)

        while not (rospy.is_shutdown() or not(self._keep_going)):
            rospy.loginfo("Waiting for incoming joint state message ...")
            rospy.sleep(0.5)
        if self._success:
            return 'success'
        else:
            return 'error'

@smach.cb_interface(input_keys=['motors'])
class EnableMotors(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success'],
                             input_keys=['motors'])
        self._pub_motor_power = rospy.Publisher("enable", std_msgs.String, latch=True)

    def execute(self, userdata):
        msg = std_msgs.String()
        for motor in userdata.motors:
            msg.data = str(motor)
            rospy.loginfo("Enabling motor '" + str(msg.data) + "'.")
            self._pub_motor_power.publish(msg)
            rospy.sleep(0.3)
        return 'success'

class GetRobotState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success'],
                             input_keys=['robot_state'],
                             output_keys=['robot_state'])
#        self._robot_state = moveit_msgs.RobotState
#        self._keep_listening = True
#        
#    def getRobotStateCB(self, msg):
#        if self._keep_listening:
#            self._robot_state = msg.robot_state
#            self._keep_listening = False
#        return
#        
#    def execute(self, userdata):
#        rospy.loginfo("Setting up subscriber on topic " + rospy.resolve_name("move_group/monitored_planning_scene"))
#        self._sub_joint_states = rospy.Subscriber("move_group/monitored_planning_scene",
#                                                  moveit_msgs.PlanningScene,
#                                                  self.getRobotStateCB)
#        
#        while not (rospy.is_shutdown() or not(self._keep_listening)):
#            rospy.loginfo("Waiting for incoming robot state message ...")
#            rospy.sleep(0.5)
#        userdata.robot_state = self._robot_state
#        return 'success'
    def execute(self, userdata):
        service_name = "get_planning_scene"
        rospy.loginfo("Waiting for '" + service_name + "' service ... ")
        rospy.wait_for_service(service_name)
        rospy.loginfo("'" + service_name + "' service available.")
        srv_client = rospy.ServiceProxy(service_name,
                                        moveit_srvs.GetPlanningScene())
        try:
            srv_req = moveit_srvs.GetPlanningSceneRequest()
            srv_req.components.components = moveit_msgs.PlanningSceneComponents.ROBOT_STATE
            srv_resp = srv_client(srv_req)
        except rospy.ServiceException, e:
            rospy.loginfo("Service did not process request: " + str(e))
        userdata.robot_state = srv_resp.scene.robot_state
        rospy.loginfo("Robot state received.")
        return 'success'


class MoveItErrorCodesParser(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success',
                                       'parsed',
                                       'no_ik_solution',
                                       'planning_failed'],
                             input_keys=['error_code'],
                             output_keys=['result'])
        self.error_code_dict = {moveit_msgs.MoveItErrorCodes.SUCCESS:'SUCCESS',
                                moveit_msgs.MoveItErrorCodes.FAILURE:'FAILURE',
                                moveit_msgs.MoveItErrorCodes.INVALID_MOTION_PLAN:'INVALID_MOTION_PLAN',
                                moveit_msgs.MoveItErrorCodes.PLANNING_FAILED:'PLANNING_FAILED',
                                moveit_msgs.MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
                                moveit_msgs.MoveItErrorCodes.CONTROL_FAILED:'CONTROL_FAILED',
                                moveit_msgs.MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:'UNABLE_TO_AQUIRE_SENSOR_DATA',
                                moveit_msgs.MoveItErrorCodes.TIMED_OUT:'TIMED_OUT',
                                moveit_msgs.MoveItErrorCodes.PREEMPTED:'PREEMPTED',
                                moveit_msgs.MoveItErrorCodes.START_STATE_IN_COLLISION:'START_STATE_IN_COLLISION',
                                moveit_msgs.MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:'START_STATE_VIOLATES_PATH_CONSTRAINTS',
                                moveit_msgs.MoveItErrorCodes.GOAL_IN_COLLISION:'GOAL_IN_COLLISION',
                                moveit_msgs.MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:'GOAL_VIOLATES_PATH_CONSTRAINTS',
                                moveit_msgs.MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:'GOAL_CONSTRAINTS_VIOLATED',
                                moveit_msgs.MoveItErrorCodes.INVALID_GROUP_NAME:'INVALID_GROUP_NAME',
                                moveit_msgs.MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:'INVALID_GOAL_CONSTRAINTS',
                                moveit_msgs.MoveItErrorCodes.INVALID_ROBOT_STATE:'INVALID_ROBOT_STATE',
                                moveit_msgs.MoveItErrorCodes.INVALID_LINK_NAME:'INVALID_LINK_NAME',
                                moveit_msgs.MoveItErrorCodes.INVALID_OBJECT_NAME:'INVALID_OBJECT_NAME',
                                moveit_msgs.MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:'FRAME_TRANSFORM_FAILURE',
                                moveit_msgs.MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:'COLLISION_CHECKING_UNAVAILABLE',
                                moveit_msgs.MoveItErrorCodes.ROBOT_STATE_STALE:'ROBOT_STATE_STALE',
                                moveit_msgs.MoveItErrorCodes.SENSOR_INFO_STALE:'SENSOR_INFO_STALE',
                                moveit_msgs.MoveItErrorCodes.NO_IK_SOLUTION:'NO_IK_SOLUTION'}

    def execute(self, userdata):
        result = pick_and_place_msgs.MoveArmResult()
        result.error_code = userdata.error_code
        if userdata.error_code in self.error_code_dict:
            result.error_message = self.error_code_dict[userdata.error_code]
            userdata.result = result
            rospy.loginfo("Move arm finished with error message: " + self.error_code_dict[userdata.error_code])
            if userdata.error_code == moveit_msgs.MoveItErrorCodes.SUCCESS:
                return 'success'
            elif userdata.error_code == moveit_msgs.MoveItErrorCodes.NO_IK_SOLUTION:
                return 'no_ik_solution'
            elif userdata.error_code == moveit_msgs.MoveItErrorCodes.PLANNING_FAILED:
                return 'planning_failed'
        else:
            result.error_message = str("Error code '" + str(userdata.error_code) + "' not in dictionary. "
                                       + "Check MoveItErrorCodes message for more information!")
            userdata.result = result
            rospy.loginfo("Error code '" + str(userdata.error_code) + "' not in dictionary. "
                          + "Check MoveItErrorCodes message for more information!")
        return 'parsed'

class change3DSensorDriverMode(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done'],
                             input_keys=['color_mode',
                                         'depth_mode',
                                         'ir_mode'],
                             output_keys=[])
        self._client = dynamic_reconfigure.client.Client("sensor_3d/driver")

    def execute(self, userdata):
        params = {'color_mode' : userdata.color_mode,
                  'depth_mode' : userdata.depth_mode,
                  'ir_mode' : userdata.ir_mode}
        config = self._client.update_configuration(params)
        rospy.sleep(2.0)
        return 'done'


class RetrieveJointStates(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'error'],
                             output_keys=['joint_states'])
        self._joint_states = sensor_msgs.JointState()
        self._joint_states_initialised = False
        self._sub_joint_states = rospy.Subscriber("joint_states", sensor_msgs.JointState, self._jointStatesCB)

    def _jointStatesCB(self, msg):
        if not self._joint_states_initialised:
            self._joint_states = msg
            self._joint_states_initialised = True
            rospy.loginfo("Joint states received.")

    def execute(self, userdata):
        self._joint_states_initialised = False
        while not self._joint_states_initialised and not rospy.is_shutdown():
            rospy.loginfo("Waiting for joint states ...")
            rospy.sleep(0.5)
        userdata.joint_states = self._joint_states
        return 'success'
