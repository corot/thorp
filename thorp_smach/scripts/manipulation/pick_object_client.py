#!/usr/bin/env python
#import sys
import argparse
import roslib; roslib.load_manifest('thorp_smach')
import rospy
import actionlib
import pick_and_place_msgs.msg as pick_and_place_msgs


def doneCb(state, result):
    rospy.loginfo('Finished in state:' + str(state))
    rospy.loginfo('With result' + str(result))
    
def activeCb():
    rospy.loginfo('Action goal just went active.')
    
def feedbackCb(feedback):
    rospy.loginfo(feedback)

def main():
    rospy.init_node('pick_object_client')
    
    parser = argparse.ArgumentParser(description='Client for Korus` pick object action server')
    parser.add_argument('object_name', type = str, help='Name of the object to be picked up')
    parser.add_argument('x', type = float, help='Object pose X-coordinate')
    parser.add_argument('y', type = float, help='Object pose Y-coordinate')
    parser.add_argument('z', type = float, help='Object pose Z-coordinate')
    args=parser.parse_args()
    
    client = actionlib.SimpleActionClient('korus/pick_object', pick_and_place_msgs.PickObjectAction)
    client.wait_for_server()
    rospy.loginfo("Action server available.")
    goal = pick_and_place_msgs.PickObjectGoal()
    goal.object_name = args.object_name
    goal.object_pose.header.stamp = rospy.Time.now()
    goal.object_pose.header.frame_id = "/base_footprint"
    goal.object_pose.pose.position.x = args.x
    goal.object_pose.pose.position.y = args.y
    goal.object_pose.pose.position.z = args.z
    goal.object_pose.pose.orientation.w = 1.0
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