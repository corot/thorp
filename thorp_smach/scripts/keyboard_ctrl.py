#!/usr/bin/env python

import rospy

import keyboard.msg as keyboard_msgs
import thorp_msgs.msg as thorp_msgs

from actionlib import *


USER_COMMANDS = {
    keyboard_msgs.Key.KEY_s: "start",
    keyboard_msgs.Key.KEY_r: "reset",
    keyboard_msgs.Key.KEY_f: "fold",
    keyboard_msgs.Key.KEY_q: "quit"
}


def keydown_cb(msg):
    rospy.loginfo("Key pressed: %s -> command: %s", msg.code, USER_COMMANDS[msg.code])

    # Creates a goal to send to the action server.
    goal = thorp_msgs.SmachCtrlGoal(command=USER_COMMANDS[msg.code])

    # Sends the goal to the action server.
    client.send_goal(goal)

# TODO: Maybe I should call this in a separated thread... just for information; DO if I ever replaced ros_keyboard by something mine where showing info continuously
#     # Waits for the server to finish performing the action.
#     client.wait_for_result()
#  
#     # Prints out the result of executing the action
#     rospy.loginfo("SM result: %s", client.get_result())  # probably empty... not implemented


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('user_commands_action_client')

        # Creates the SimpleActionClient, passing the type of the action
        # (SmachCtrlAction) to the constructor.
        client = SimpleActionClient('user_commands_action_server', thorp_msgs.SmachCtrlAction)
    
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        rospy.Subscriber('object_manipulation_keyboard/keydown', keyboard_msgs.Key, keydown_cb)

        rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
