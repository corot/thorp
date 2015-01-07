#!/usr/bin/env python
from thorp_smach.pick_and_place_tools import misc_tools
from thorp_smach.state_machines import arm_calibration_sm
from thorp_smach.state_machines.state_machines_imports import *


'''
Arm calibration
'''
def main():
    rospy.init_node('arm_calibration')
    
    sm_arm_calibration = smach.StateMachine(outcomes=['calibrated', 'calibration_failed'])

    with sm_arm_calibration:
        
        sm_arm_calibration.userdata.motors = ["torso_turn",
                                              "torso_lift",
                                              "shoulder",
                                              "elbow",
                                              "wrist",
                                              "gripper",
                                              "head_pan",
                                              "head_tilt"]
        smach.StateMachine.add('EnableMotors',
                               misc_tools.EnableMotors(),
                               remapping={'motors':'motors'}, 
                               transitions={'success':'ArmCalibration'})

        sm_arm = arm_calibration_sm.createSM()
        smach.StateMachine.add('ArmCalibration',
                               sm_arm,
                               transitions={'calibrated':'calibrated',
                                            'calibration_failed':'calibration_failed'})

    sis = smach_ros.IntrospectionServer('introspection_server', sm_arm_calibration, '/SM_ROOT')
    sis.start()
    
    sm_arm_calibration.execute()
    
    sis.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

