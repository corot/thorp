#!/usr/bin/env python
import roslib; roslib.load_manifest('thorp_smach')
import rospy
import smach
import smach_ros
from smach_ros import ServiceState
import goo_msgs.srv as goo_srvs
from thorp_smach.calibration_tools import actuator
from thorp_smach.pick_and_place_tools import misc_tools

def createSM():
    
    sm_head_calibration = smach.StateMachine(outcomes=['head_calibrated', 'head_calibration_failed'])
    
    with sm_head_calibration:
        
        sm_head_calibration.userdata.motors = ["head_pan", "head_tilt"]
        
        sm_head_calibration.userdata.forward_direction = 1
        sm_head_calibration.userdata.backward_direction = -1
        sm_head_calibration.userdata.get_one_limit_only_true = True
        sm_head_calibration.userdata.get_one_limit_only_false = False
        sm_head_calibration.userdata.complete_calibration_true = True
        sm_head_calibration.userdata.complete_calibration_false = False
        sm_head_calibration.userdata.encoder_goal_true = True
        sm_head_calibration.userdata.encoder_goal_false = False
        sm_head_calibration.userdata.move_to_hard_limit_true = True
        sm_head_calibration.userdata.move_to_hard_limit_false = False
        sm_head_calibration.userdata.move_to_zero_true = True
        sm_head_calibration.userdata.move_to_zero_false = False
        
        sm_head_calibration.userdata.actuator_head_pan_name = "head_pan"
        sm_head_calibration.userdata.actuator_head_pan_array_pos = 2
        sm_head_calibration.userdata.actuator_head_pan_max_current = 200
        sm_head_calibration.userdata.actuator_head_pan_angle_ratio = 2 # min/max the same; (f(x) = (max - min)/(max)
        sm_head_calibration.userdata.actuator_head_pan_speed = 0.02
        sm_head_calibration.userdata.actuator_head_pan_encoder_ticks = 215450
        sm_head_calibration.userdata.actuator_head_pan_limit_min = 0
        sm_head_calibration.userdata.actuator_head_pan_limit_max = 0
        sm_head_calibration.userdata.actuator_head_pan_encoder_centre = 0
        
        sm_head_calibration.userdata.actuator_head_tilt_name = "head_tilt"
        sm_head_calibration.userdata.actuator_head_tilt_array_pos = 3
        sm_head_calibration.userdata.actuator_head_tilt_max_current = 200
        sm_head_calibration.userdata.actuator_head_tilt_angle_ratio = 3.875 # min = -92, max = 32
        sm_head_calibration.userdata.actuator_head_tilt_speed = 0.02
        sm_head_calibration.userdata.actuator_head_tilt_encoder_ticks = 215450
        sm_head_calibration.userdata.actuator_head_tilt_limit_min = 0
        sm_head_calibration.userdata.actuator_head_tilt_limit_max = 0
        sm_head_calibration.userdata.actuator_head_tilt_encoder_centre = 0
        
#        smach.StateMachine.add('EnableMotors',
#                               misc_tools.EnableMotors(),
#                               remapping={'motors':'motors'}, 
#                               transitions={'success':'CalibrateHeadPan'})
        
        smach.StateMachine.add('CalibrateHeadPan', actuator.CalibrateActuator(),
                               transitions={'success':'CalibrateHeadTilt',
                                            'error':'head_calibration_failed'},
                               remapping={'actuator_name':'actuator_head_pan_name',
                                          'actuator_array_pos':'actuator_head_pan_array_pos',
                                          'actuator_max_current':'actuator_head_pan_max_current',
                                          'actuator_angle_ratio':'actuator_head_pan_angle_ratio',
                                          'actuator_speed':'actuator_head_pan_speed',
                                          'actuator_direction':'forward_direction',
                                          'encoder_ticks':'actuator_head_pan_encoder_ticks',
                                          'actuator_limit_min':'actuator_head_pan_limit_min',
                                          'actuator_limit_max':'actuator_head_pan_limit_max',
                                          'actuator_encoder_centre':'actuator_head_pan_encoder_centre',
                                          'move_to_zero':'move_to_zero_true',
                                          'get_one_limit_only':'get_one_limit_only_false',
                                          'complete_calibration':'complete_calibration_true'})
        
        smach.StateMachine.add('CalibrateHeadTilt',
                               actuator.CalibrateActuator(),
                               transitions={'success':'ResetHeadPanEncoder',
                                            'error':'head_calibration_failed'},
                               remapping={'actuator_name':'actuator_head_tilt_name',
                                          'actuator_array_pos':'actuator_head_tilt_array_pos',
                                          'actuator_max_current':'actuator_head_tilt_max_current',
                                          'actuator_angle_ratio':'actuator_head_tilt_angle_ratio',
                                          'actuator_speed':'actuator_head_tilt_speed',
                                          'actuator_direction':'backward_direction',
                                          'encoder_ticks':'actuator_head_tilt_encoder_ticks',
                                          'actuator_limit_min':'actuator_head_tilt_limit_min',
                                          'actuator_limit_max':'actuator_head_tilt_limit_max',
                                          'actuator_encoder_centre':'actuator_head_tilt_encoder_centre',
                                          'move_to_zero':'move_to_zero_true',
                                          'get_one_limit_only':'get_one_limit_only_false',
                                          'complete_calibration':'complete_calibration_true'})
        
        smach.StateMachine.add('ResetHeadPanEncoder',
                               ServiceState('goo/reset_encoders',
                                            goo_srvs.ResetEncoders,
                                            request = goo_srvs.ResetEncodersRequest("head_pan")),
                                            transitions={'succeeded':'ResetHeadTiltEncoder',
                                                         'preempted':'head_calibration_failed',
                                                         'aborted':'head_calibration_failed',})
        
        smach.StateMachine.add('ResetHeadTiltEncoder', 
                               ServiceState('goo/reset_encoders',
                                            goo_srvs.ResetEncoders,
                                            request = goo_srvs.ResetEncodersRequest("head_tilt")),
                                            transitions={'succeeded':'head_calibrated',
                                                         'preempted':'head_calibration_failed',
                                                         'aborted':'head_calibration_failed',})
    return sm_head_calibration