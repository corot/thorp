#! /usr/bin/env python

import math
import rospy

import actionlib

import thorp_msgs.msg as thorp_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs


def move_to_target_client(value):
    # Creates the SimpleActionClient, passing the type of the action
    # (MoveToTargetAction) to the constructor.
    client = actionlib.SimpleActionClient('/move_to_target', thorp_msgs.MoveToTargetAction)

    # Waits until the action server has started up and started listening for goals
    client.wait_for_server()

    # Depending on value's type, create a goal to send to the action server.
    if type(value) is list:
        target = sensor_msgs.JointState()
        target.name = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint']
        target.position = value
        goal = thorp_msgs.MoveToTargetGoal(target_type=thorp_msgs.MoveToTargetGoal.JOINT_STATE, joint_state=target)
    elif type(value) is str:
        goal = thorp_msgs.MoveToTargetGoal(target_type=thorp_msgs.MoveToTargetGoal.NAMED_TARGET, named_target=value)
    elif type(value) is tuple:
        target = geometry_msgs.PoseStamped()
        target.pose.position.x = value[0]
        target.pose.position.y = value[1]
        target.pose.position.z = value[2]
        target.header.frame_id = 'arm_shoulder_lift_servo_link'
        goal = thorp_msgs.MoveToTargetGoal(target_type=thorp_msgs.MoveToTargetGoal.POSE_TARGET, pose_target=target)
    else:
        print "Invalid target value: ", value
        return None

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_arm_ctrl_servers')
        result = move_to_target_client([0.0, math.pi/2, 0.0, 0.0])
        print "Result: ",  result
        result = move_to_target_client('resting')
        print "Result: ",  result
        result = move_to_target_client((0.08, 0.0, 0.101))
        print "Result: ",  result
        result = move_to_target_client((0.10, 0.0, 0.101))
        print "Result: ",  result
        result = move_to_target_client((0.15, 0.0, 0.101))
        print "Result: ",  result
        result = move_to_target_client((0.20, 0.0, 0.101))
        print "Result: ",  result
        result = move_to_target_client((0.25, 0.0, 0.101))
        print "Result: ",  result
        result = move_to_target_client((0.29, 0.0, 0.101))
        print "Result: ",  result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
