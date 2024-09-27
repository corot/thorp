#!/usr/bin/env python

"""
Movie director: place RViz camera on different locations whenever the current SMACH state is listed in the script
Author:
    Jorge Santos
"""

import re
import sys

import yaml
import copy
import rospy
import pickle
import base64

from thorp_msgs.msg import BTNodeStatus
from smach_ros.introspection import STATUS_TOPIC
from smach_msgs.msg import SmachContainerStatus
from geometry_msgs.msg import Point, Vector3, PoseStamped
from view_controller_msgs.msg import CameraPlacement


def place_camera(target_frame, focus_point, eye_point):
    cp = CameraPlacement()
    cp.target_frame = target_frame

    cp.focus.point = focus_point
    cp.focus.header.frame_id = target_frame

    cp.eye.point = eye_point
    cp.eye.header.frame_id = target_frame

    cp.up.vector = Vector3(0, 0, 1)
    cp.up.header.frame_id = target_frame

    cp.time_from_start = rospy.Duration(2)
    cp.interpolation_mode = CameraPlacement.SPHERICAL
    cp.interaction_disabled = False
    # Unused options:
    #   cp.mouse_interaction_mode
    #   cp.allow_free_yaw_axis

    cp_pub.publish(cp)


def parse_state(state, camera_instructions, userdata=None):
    try:
        target_frame = camera_instructions['frame']

        focus = camera_instructions['focus']
        if isinstance(focus, list):
            focus_point = Point(*focus)
        elif isinstance(focus, str):
            focus_point = userdata[focus].pose.position
        elif isinstance(focus, dict):
            focus_point = rospy.wait_for_message(focus['topic'], PoseStamped).pose.position
        else:
            raise KeyError("Invalid focus type: %s" % str(type(focus)))

        eye = camera_instructions['eye']
        if isinstance(eye, list):
            eye_point = copy.deepcopy(focus_point)
            eye_point.x += eye[0]
            eye_point.y += eye[1]
            eye_point.z += eye[2]
        # elif isinstance(eye, (int, float)):  TODO, if ever needed
        #     eye_point = copy.deepcopy(focus_point)
        #     eye_point.y -= 0.1
        #     eye_point.z += eye
        # elif isinstance(eye, str):
        #     eye_point = userdata[eye].pose.position
        else:
            raise KeyError("Invalid eye type: %s" % str(type(eye)))
        return target_frame, focus_point, eye_point
    except KeyError as ke:
        rospy.logerr("Invalid camera instructions for state %s: %s", state, str(ke))
        raise ke


def smach_status_cb(msg):
    try:
        # search for re ['<state name>'] on info field
        # (it looks something like "(<smach.user_data.UserData object at 0x7f0889b1e490>, ['PICKUP_OBJECT']), {}")
        match = re.search(r'\[\'([A-Za-z0-9_]+)\'\]', msg.info)
        if match is None:
            return
        current_state = match.group(1)
        camera_instructions = script[current_state]
        t0 = rospy.get_time()
        # userdata comes base64-compressed and pickle-serialized; deserialization can take long for heavy userdata
        userdata = pickle.loads(base64.b64decode(msg.local_data))
        rospy.loginfo("Placing camera for state %s (deserialization took %.3f s)", current_state, rospy.get_time() - t0)
        place_camera(*parse_state(current_state, camera_instructions, userdata))
    except (ImportError, pickle.PicklingError) as ude:
        rospy.logerr("Parse userdata error: %s", str(ude))
        # TODO: probably I don't need it, but smach viz does the following:
        #   This will only happen once for each package
        #   modulename = ie.args[0][16:]
        #   packagename = modulename[0:modulename.find('.')]
        #   roslib.load_manifest(packagename)
        #   self._local_data._data = pickle.loads(msg.local_data)
    except KeyError as ke:
        pass  # normal; most states won't be listed in the script


def bt_status_cb(msg):
    try:
        if msg.prev_status == BTNodeStatus.IDLE and msg.status == BTNodeStatus.RUNNING:
            current_state = msg.name
            camera_instructions = script[current_state]
            rospy.loginfo("Placing camera for state %s", current_state)
            place_camera(*parse_state(current_state, camera_instructions))
    except KeyError as ke:
        pass  # normal; most states won't be listed in the script

def get_value_from_path(msg, path):
    keys = path.split('.')
    value = msg
    try:
        for key in keys:
            value = getattr(value, key)
        return value
    except AttributeError:
        rospy.logerr(f"Path '{path}' not found in message.")
        return None

def callback_factory(topic_name, path):
    def callback(msg):
        topic_values[topic_name] = msg
        print()
        print(topic_name)
        print()
        print(msg)
        print()
        # rospy.loginfo(f"Extracted value for {topic_name}: {value}")
        # value = get_value_from_path(msg, path)
        # if value is not None:
        #     topic_values[topic_name] = value
        #     rospy.loginfo(f"Extracted value for {topic_name}: {value}")
    return callback

def create_subscribers(script):
    for key, entry in script.items():
        if isinstance(entry['focus'], dict):
            topic = entry['focus'].get('topic')
            path = entry['focus'].get('path')
            # if not topic or not path:
            #     rospy.logerr(f"Invalid entry in YAML: {key}")
            #     continue

            global topic_values
            topic_values[topic] = None

            rospy.Subscriber(topic, PoseStamped, callback_factory(topic, path))


if __name__ == "__main__":
    rospy.init_node("movie_director")

    # read movie script yaml file
    script_path = rospy.get_param('~script_path')
    with open(script_path) as script_file:
        # The FullLoader parameter handles the conversion from YAML scalar values to Python the dictionary format
        script = yaml.load(script_file, Loader=yaml.FullLoader)

    if not script:
        rospy.logerr("Cannot read script file %s", script_path)
        sys.exit(-1)

    cp_pub = rospy.Publisher('rviz/camera_placement', CameraPlacement, queue_size=1)

    # debug
    # for name, state in script.items():
    #     target_frame, focus_point, eye_point = parse_state(state)
    #     place_camera(target_frame, focus_point, eye_point)

    topic_values = {}
    create_subscribers(script)

    server_name = rospy.get_param('~app_name')
    rospy.Subscriber(server_name + STATUS_TOPIC, SmachContainerStatus, smach_status_cb, queue_size=5)
    rospy.Subscriber(server_name + '/bt_status', BTNodeStatus, bt_status_cb, queue_size=50)

    rospy.spin()
