#!/usr/bin/env python

"""
Add interactive markers to move Gazebo models
Adapted from https://github.com/ros-visualization/visualization_tutorials/blob/noetic-devel/interactive_marker_tutorials/scripts/basic_controls.py
Author:
    Jorge Santos
"""

import sys
import copy
import math
import rospy

from interactive_markers.interactive_marker_server import InteractiveMarker, InteractiveMarkerServer, \
                                                          InteractiveMarkerFeedback
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelState, ModelStates

from thorp_toolkit.common import wait_for_sim_time
from thorp_toolkit.geometry import quaternion_msg_from_yaw, yaw, normalize_quaternion, pose3d2str

server = None
menu_handler = MenuHandler()
br = None
counter = 0


def process_feedback(feedback):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.logdebug(s + ": button click" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.logdebug(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.logdebug(s + ": pose changed to " + pose3d2str(feedback.pose))
        state_pub.publish(ModelState(feedback.marker_name, feedback.pose, Twist(), feedback.header.frame_id))
        pose_pub.publish(PoseStamped(feedback.header, feedback.pose))
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.logdebug(s + ": mouse down" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.logdebug(s + ": mouse up" + mp + ".")
    server.applyChanges()


def make_model_marker(model):
    marker = Marker()
    marker.type = Marker.MESH_RESOURCE
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    model_type = '_'.join(model.split('_')[:-1])  # remove instance index to get the model type
    marker.mesh_resource = "package://thorp_simulation/worlds/gazebo/models/" + model_type + "/meshes/Cat_v1_l3.obj"
    marker.mesh_use_embedded_materials = True
    # TODO: get mesh path and mesh pose from the model SDF file to make this generic and avoid the hack below
    if model_type == 'cat_black':
        marker.pose.position.x = -0.09
        marker.pose.orientation = quaternion_msg_from_yaw(math.pi/2.0)
    if model_type == 'cat_orange':
        marker.pose.orientation = quaternion_msg_from_yaw(math.pi/2.0)
    return marker


def make_model_control(msg, model):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(make_model_marker(model))
    msg.controls.append(control)
    return control


def save_marker(int_marker):
    server.insert(int_marker, process_feedback)


#####################################################################
# Marker Creation

def make_interactive_marker(pose, model):
    int_marker = InteractiveMarker()
    int_marker.name = model
    int_marker.description = model
    int_marker.header.frame_id = "map"
    int_marker.pose = pose
    int_marker.pose.position.z = max(int_marker.pose.position.z, 0.01)  # ensure marker is above ground
    int_marker.pose.orientation = quaternion_msg_from_yaw(yaw(pose))  # discard all but yaw to ensure marker is flat
    int_marker.scale = 1

    make_model_control(int_marker, model)

    control = InteractiveMarkerControl()
    control.name = "drag"
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    normalize_quaternion(control.orientation)
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    int_marker.controls.append(copy.deepcopy(control))
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    server.insert(int_marker, process_feedback)
    menu_handler.apply(server, int_marker.name)
    return int_marker


def model_states_cb(msg):
    for index, model_name in enumerate(msg.name):
        if model_name in target_models:
            if server.get(model_name) is None:
                make_interactive_marker(msg.pose[index], model_name)
                rospy.loginfo("Interactive marker added at %s for model %s", pose3d2str(msg.pose[index]), model_name)
            else:
                server.setPose(model_name, msg.pose[index])
    server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("model_markers")

    server = InteractiveMarkerServer("model_markers")

    menu_handler.insert("First Entry", callback=process_feedback)
    menu_handler.insert("Second Entry", callback=process_feedback)
    sub_menu_handle = menu_handler.insert("Submenu")
    menu_handler.insert("First Entry", parent=sub_menu_handle, callback=process_feedback)
    menu_handler.insert("Second Entry", parent=sub_menu_handle, callback=process_feedback)

    state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=1)
    pose_pub = rospy.Publisher("target_object_pose", PoseStamped, queue_size=1)

    if len(sys.argv) < 2:
        rospy.logwarn("No target models provided; nothing to do")
        print("Usage: model_markers.py <string of space-separated model names>")
        sys.exit(0)
    target_models = sys.argv[1]
    if type(target_models) != list:
        target_models = target_models.split()

    # wait for clock to start before listening for models (we unpause simulation after spawning the robot)
    wait_for_sim_time()
    model_states_sub = rospy.Subscriber("gazebo/model_states", ModelStates, model_states_cb, queue_size=1)

    rospy.spin()
