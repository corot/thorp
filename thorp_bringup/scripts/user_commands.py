#!/usr/bin/env python

"""
Listen for user commands on RViz and forward them to the SMACH container wrapper for execution.
Author:
    Jorge Santos
"""

import rospy

import thorp_msgs.msg as thorp_msgs

from std_msgs.msg import ColorRGBA
from actionlib import SimpleActionClient, GoalStatus

from jsk_rviz_plugins.msg import OverlayText
from jsk_rviz_plugins.srv import EusCommand, EusCommandResponse

from thorp_toolkit.visualization import Visualization


def handle_user_command(request):
    cmd_msg = f"User command: {request.command}"
    rospy.loginfo(cmd_msg)

    if request.command == 'exit':
        # Force exit; if this node required, that will shut down the whole app
        global exit_requested
        exit_requested = True
        exit_msg = "Shutting down app"
        rospy.logwarn(exit_msg)
        cmd_pub.publish(Visualization.create_overlay_text(20, ColorRGBA(1.0, 0.65, 0.0, 1.0), exit_msg, 12))
    else:
        # Send user command goal and wait for server response; all commands will require executing several
        # actions, so if the server answers very fast, probably the app doesn't support the selected command
        client.send_goal(thorp_msgs.UserCommandGoal(command=request.command))
        if client.wait_for_result(rospy.Duration(0.5)) and client.get_state() != GoalStatus.SUCCEEDED:
            assert client.get_result().outcome == 'invalid_command'
            err_msg = f"{request.command} command not supported"
            rospy.logerr(err_msg)
            cmd_pub.publish(Visualization.create_overlay_text(20, ColorRGBA(1.0, 0.0, 0.0, 1.0), err_msg, 12))
            return None  # RViz will show an error
        cmd_pub.publish(Visualization.create_overlay_text(20, ColorRGBA(1.0, 1.0, 1.0, 1.0), cmd_msg, 12))
    return EusCommandResponse()


if __name__ == '__main__':
    rospy.init_node('user_commands')

    # Creates action client to execute user commands
    client = SimpleActionClient('user_commands_action_server', thorp_msgs.UserCommandAction)
    client.wait_for_server()

    # Handle user commands coming from RViz and show current one as an overly
    cmd_pub = rospy.Publisher('rviz/user_command', OverlayText, queue_size=1)
    cmd_srv = rospy.Service('eus_command', EusCommand, handle_user_command)

    exit_requested = False
    while not exit_requested and not rospy.is_shutdown():
        rospy.sleep(0.1)
