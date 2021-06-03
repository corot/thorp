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
import cPickle

from smach_ros import introspection
from smach_msgs.msg import SmachContainerStatus
from geometry_msgs.msg import Point, Vector3
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
        # userdata comes pickle-serialized;   WARN: deserialization can take a lot for heavy userdata!
        userdata = cPickle.loads(msg.local_data)
        rospy.loginfo("Placing camera for state %s (deserialization took %.3f s)", current_state, rospy.get_time() - t0)
        place_camera(*parse_state(current_state, camera_instructions, userdata))
    except (ImportError, cPickle.PicklingError) as ude:
        rospy.logerr("Parse userdata error: %s", str(ude))
        # TODO: probably I don't need it, but smach viz does the following:
        #   This will only happen once for each package
        #   modulename = ie.args[0][16:]
        #   packagename = modulename[0:modulename.find('.')]
        #   roslib.load_manifest(packagename)
        #   self._local_data._data = pickle.loads(msg.local_data)
    except KeyError as ke:
        rospy.logdebug("State %s not found in script", str(ke))  # normal; most states won't be listed in the script!


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

    server_name = rospy.get_param('~app_name')
    status_topic = server_name + introspection.STATUS_TOPIC
    cs_sub = rospy.Subscriber(status_topic, SmachContainerStatus, smach_status_cb, queue_size=5)

    rospy.spin()
