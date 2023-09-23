import rospy
import actionlib

import rosgraph_msgs.msg as rosgraph_msgs

import std_srvs.srv as std_srvs
import mbf_msgs.msg as mbf_msgs


def pause_gazebo():
    if not hasattr(pause_gazebo, 'srv'):
        pause_gazebo.srv = rospy.ServiceProxy('gazebo/pause_physics', std_srvs.Empty)
        pause_gazebo.srv.wait_for_service(0.1)
    pause_gazebo.srv()


def resume_gazebo():
    if not hasattr(resume_gazebo, 'srv'):
        resume_gazebo.srv = rospy.ServiceProxy('gazebo/unpause_physics', std_srvs.Empty)
        resume_gazebo.srv.wait_for_service(0.1)
    resume_gazebo.srv()


def wait_for_sim_time():
    """
    In sim, wait for clock to start (I start gazebo paused, so smach action clients start waiting at time 0,
    but first clock marks ~90s, after spawner unpauses physics)
    """
    if rospy.get_param('/use_sim_time', False):
        if not rospy.wait_for_message('/clock', rosgraph_msgs.Clock, rospy.Duration(60)):
            rospy.logfatal("No clock msgs after 60 seconds, being use_sim_time true")
            return False
    return True


def wait_for_mbf():
    """
    Wait for Move Base Flex's move_base action (the last to be started) getting available
    """
    if rospy.get_param('/move_base_flex', False):
        mb_ac = actionlib.SimpleActionClient("/move_base_flex/move_base", mbf_msgs.MoveBaseAction)
        if not mb_ac.wait_for_server(rospy.Duration(30)):
            rospy.logwarn("Move Base Flex not available after 30 seconds")
            return False
    return True
