cmake_minimum_required(VERSION 3.0.2)
project(thorp_agent)

find_package(catkin REQUIRED COMPONENTS rospy actionlib actionlib_msgs thorp_msgs)

catkin_python_setup()
catkin_package()

# No catkin_install_python for nodes/agent.py: its devel relay would run the node under python
# 3.8, and it needs the /opt/thorp-agent venv its shebang names, so rosrun runs it from source
