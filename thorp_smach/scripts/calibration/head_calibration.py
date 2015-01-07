#!/usr/bin/env python
from thorp_smach.pick_and_place_tools import misc_tools
from thorp_smach.state_machines import head_calibration_sm
from thorp_smach.state_machines.state_machines_imports import *


'''
 Head calibration
'''
def main():
    rospy.init_node('head_calibration')
    
    sm_head_calibration = smach.StateMachine(outcomes=['calibrated', 'calibration_failed'])

    with sm_head_calibration:
        
        sm_head_calibration.userdata.motors = ["head_pan",
                                               "head_tilt"]
        smach.StateMachine.add('EnableMotors',
                               misc_tools.EnableMotors(),
                               remapping={'motors':'motors'}, 
                               transitions={'success':'HeadCalibration'})

        sm_head = head_calibration.createSM()
        smach.StateMachine.add('HeadCalibration',
                               sm_head,
                               remapping={'motors':'motors'}, 
                               transitions={'head_calibrated':'calibrated',
                                            'head_calibration_failed':'calibration_failed'})

    sis = smach_ros.IntrospectionServer('introspection_server', sm_head_calibration, '/SM_ROOT')
    sis.start()
    
    sm_head_calibration.execute()
    
    sis.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

