#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""The aliencontrol ROS node

This ROS node allows controlling the execution of external programs from within
a ROS node. "External" refers to binary files and scripts that are not ROS
nodes, for example bash scripts and system programs. "To control" means starting
and stopping these applications.

In order to start an external process, pass the following commands to
aliencontrol.

Examples:
    # Start an aliencontrol node that launches top inside the current terminal.
    $ rosrun aliencontrol aliencontrol "top"

    # Launch top in a separate terminal.
    $ rosrun aliencontrol aliencontrol "xterm -e top"

These commands spawn a aliencontrol ROS node. The node terminates once you quit
the external application. If you terminate the node, aliencontrol it will
automatically kill the external application before shutting down.
"""

# Import the required system libraries.
import rospy
import sys
import argparse
import os
import subprocess
import signal

# Start the aliencontrol ROS node.
if __name__ == '__main__':
    # Register the node with the roscore.
    rospy.init_node('aliencontrol', anonymous=True)

    # Extract the command to execute from the input arguments. Accept both a ROS
    # private parameter (like in "rosrun aliencontrol aliencontrol _cmd:=top")
    # or an anonymous parameter (like in
    # "rosrun aliencontrol aliencontrol top").
    cmd = ''
    if rospy.has_param('~cmd'):
        # Read the parameter from the ROS parameter server.
        cmd = rospy.get_param('~cmd')
    else:
        # Read the anonymous parameter.
        args = rospy.myargv(argv=sys.argv)

        # Delete the first input argument, which is always the path of this
        # script.
        del (args[0])

        # Extract the command.
        parser = argparse.ArgumentParser(
            description='Control external program from within a ROS node.')
        parser.add_argument('cmd', help='external command to execute')
        cmd = parser.parse_args(args).cmd

    # Start the external application as a process group. In this way, we make
    # sure to control not only the external process, but all of its
    # subprocesses.
    rospy.loginfo('Executing command \"%s\" ...', cmd)
    alien = subprocess.Popen(args=cmd, shell=True, preexec_fn=os.setsid)


    # Define the shutdown handler.
    def shutdown():
        """Shutdown handler

        Shuts down the external application using a sequence of signals.
        """
        rospy.loginfo('Shutting down aliencontrol ...')

        # Shut down the external application, if it is still running.
        try:
            if alien.poll() == None:
                # Retrieve the ID of the process group that represents the
                # external application.
                pgid = os.getpgid(alien.pid)

                # Ask the process to gracefully shut down using SIGINT.
                rospy.loginfo(
                    'Sending SIGINT to external process (PGID=%i, CMD=%s) ...',
                    pgid, cmd)
                os.killpg(pgid, signal.SIGINT)

                # Give the process some time to shut down.
                wait_duration = rospy.Duration.from_sec(5.0)
                rospy.loginfo(
                    'Waiting for %i s until escalating to SIGTERM ...',
                    wait_duration.secs)
                wait_time_end = rospy.Time.now() + wait_duration
                rate = rospy.Rate(100)
                while wait_time_end > rospy.Time.now():
                    rate.sleep()
                    if alien.poll() != None:
                        rospy.loginfo('External process is shut down.')
                        return

                # Ask the process to shut down using SIGTERM.
                rospy.logwarn(
                    'Sending SIGTERM to external process (PGID=%i, CMD=%s) ...',
                    pgid, cmd)
                os.killpg(pgid, signal.SIGTERM)

                # Give the process some more time to shut down.
                rospy.loginfo('Waiting for %i s until killing the process ...',
                              wait_duration.secs)
                wait_time_end = rospy.Time.now() + wait_duration
                while wait_time_end > rospy.Time.now():
                    rate.sleep()
                    if alien.poll() != None:
                        rospy.loginfo('External process is shut down.')
                        return

                # If the process is still running, kill it.
                rospy.logwarn(
                    'Sending SIGKILL to external process (PGID=%i, CMD=%s) ...',
                    pgid, cmd)
                os.killpg(pgid, signal.SIGKILL)
        except Exception as e:
            rospy.logerr(e)
            raise


    # Register the shutdown handler.
    rospy.on_shutdown(shutdown)

    # Poll the external process to see if it is still active. If not, exit.
    rate = rospy.Rate(10)
    while (not rospy.is_shutdown()) and (alien.poll() == None):
        rate.sleep()
