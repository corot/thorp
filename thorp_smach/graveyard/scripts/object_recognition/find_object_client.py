#!/usr/bin/env python

import sys
import argparse

# ros basics& smach
import roslib; roslib.load_manifest('thorp_smach')
import rospy
import actionlib
import pick_and_place_msgs.msg as pick_and_place_msgs
from geometry_msgs.msg import PointStamped

def doneCb(state, result):
    rospy.loginfo('Finished in state:' + str(state))
#    rospy.loginfo('With result' + str(result))
    
def activeCb():
    rospy.loginfo('Action goal just went active.')
    
def feedbackCb(feedback):
    rospy.loginfo(feedback)

def main():
    rospy.init_node('find_object_client')
    
    parser = argparse.ArgumentParser(description='Client for Korus` find object action server')
    parser.add_argument('pos_x', type = float, help='X value of the table position')
    parser.add_argument('pos_y', type = float, help='Y value of the table position')
    parser.add_argument('pos_z', type = float, help='Z value of the table position')
    parser.add_argument('--look_around', dest='look_around', action='store_true', default=False, help='Look around to find object')
    args=parser.parse_args()
    
    table_position = PointStamped()
    table_position.header.stamp = rospy.Time.now()
    table_position.header.frame_id = "base_footprint"
    table_position.point.x = args.pos_x
    table_position.point.y = args.pos_y
    table_position.point.z = args.pos_z
    
    rospy.loginfo("Will try to find an object at the given position: x = " + str(table_position.point.x)
                  + ", y = " + str(table_position.point.y) + ", z = " + str(table_position.point.z) + ")")
    
    client = actionlib.SimpleActionClient('korus/find_object', pick_and_place_msgs.FindObjectAction)
    client.wait_for_server()
    rospy.loginfo("Action server available.")
    goal = pick_and_place_msgs.FindObjectGoal()
    goal.table_position = table_position
    goal.min_confidence = 0.001
    goal.look_around = args.look_around
    rospy.loginfo("Sending action goal ...")
    client.send_goal(goal, doneCb, activeCb, feedbackCb)
    rospy.loginfo("Waiting for result ...")
    if client.wait_for_result(rospy.Duration.from_sec(30.0)):
        rospy.loginfo("Action finished in time.")
    else:
        rospy.loginfo("Action took longer than expected.")
    rospy.loginfo("Action result:")
    rospy.loginfo(client.get_result())
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass