#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the gripper command action,
# including the goal message and the result message.
import control_msgs.msg

def gripper_cmd_client(position):
    # Creates the SimpleActionClient, passing the type of the action
    # (GripperCommandAction) to the constructor.
    client = actionlib.SimpleActionClient('/gripper_controller/gripper_action',
                                           control_msgs.msg.GripperCommandAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    command = control_msgs.msg.GripperCommand()
    command.position = position
    goal = control_msgs.msg.GripperCommandGoal(command)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A GripperCommandResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('gripper_cmd_client_py')
        position = rospy.get_param('~position', 0.0)
        result = gripper_cmd_client(position)
        print("Result: ", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
