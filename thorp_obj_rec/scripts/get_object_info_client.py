#!/usr/bin/env python
"""
This file is a ROS service client for retrieving information about an object
"""
import actionlib
import argparse
import rospy
import sys
import object_recognition_msgs.msg as object_recognition_msgs
import object_recognition_msgs.srv as object_recognition_srvs


def on_result(status, result):
    print result

def main():
    parser = argparse.ArgumentParser(description='This file is a ROS service client for retrieving information about an object')
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node('object_info_client')
    
    rospy.wait_for_service('get_object_info')
    get_object_info_srv = rospy.ServiceProxy('get_object_info', object_recognition_srvs.GetObjectInformation)
    try:
        object_type = object_recognition_msgs.ObjectType
        object_type.key = '18800'
        object_type.db = '{"host":"localhost","module":"object_recognition_tabletop","name":"household_objects-0.6","password":"yujin","port":"5432","type":"ObjectDbSqlHousehold","user":"yujin"'
        req = object_recognition_srvs.GetObjectInformationRequest(object_type)
        resp = get_object_info_srv(req)
    except rospy.ServiceException, e:
      print "Service did not process request: %s"%str(e)
      
    print resp


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass