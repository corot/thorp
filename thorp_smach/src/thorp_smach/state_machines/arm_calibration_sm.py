#!/usr/bin/env python
from state_machines_imports import *
from thorp_smach.calibration_tools import actuator
from thorp_smach.calibration_tools.msg_imports import *
from thorp_smach.pick_and_place_tools import misc_tools
from thorp_smach.pick_and_place_tools.msg_imports import *

def createSM():

    sm_arm_calibration = smach.StateMachine(outcomes=['calibrated', 'calibration_failed'])

    with sm_arm_calibration:

        sm_arm_calibration.userdata.wait_2sec = 2.0

        '''
        Torso calibration
        '''
#        sm_torso = smach.StateMachine(outcomes=['torso_calibrated',
#                                                'torso_calibration_failed'])
#        
#        with sm_torso:
#            
#            sm_torso.userdata.motor_torso_turn = ["torso_turn"]
#            sm_torso.userdata.forward_direction = 1
#            sm_torso.userdata.backward_direction = -1
#            sm_torso.userdata.get_one_limit_only_true = True
#            sm_torso.userdata.get_one_limit_only_false = False
#            sm_torso.userdata.complete_calibration_true = True
#            sm_torso.userdata.complete_calibration_false = False
#            sm_torso.userdata.encoder_goal_true = True
#            sm_torso.userdata.encoder_goal_false = False
#            sm_torso.userdata.move_to_hard_limit_true = True
#            sm_torso.userdata.move_to_hard_limit_false = False
#            sm_torso.userdata.move_to_zero_true = True
#            sm_torso.userdata.move_to_zero_false = False
#            sm_torso.userdata.actuator_torso_turn_name = "torso_turn"
#            sm_torso.userdata.actuator_torso_turn_array_pos = 6
#            sm_torso.userdata.actuator_torso_turn_max_current = 80
#            sm_torso.userdata.actuator_torso_turn_angle_ratio = 2.1625 # = (max - min) / max with min = -186, max = 160
#            sm_torso.userdata.actuator_torso_turn_speed = 0.015
#            sm_torso.userdata.actuator_torso_turn_encoder_ticks = 276000
#            sm_torso.userdata.actuator_torso_turn_limit_min = 0
#            sm_torso.userdata.actuator_torso_turn_limit_max = 0
#            sm_torso.userdata.actuator_torso_turn_encoder_centre = 0
#            
#            smach.StateMachine.add('CalibrateTorsoTurn',
#                                   actuator.CalibrateActuator(), 
#                                   remapping={'actuator_name':'actuator_torso_turn_name',
#                                              'actuator_array_pos':'actuator_torso_turn_array_pos',
#                                              'actuator_max_current':'actuator_torso_turn_max_current',
#                                              'actuator_angle_ratio':'actuator_torso_turn_angle_ratio',
#                                              'actuator_speed':'actuator_torso_turn_speed',
#                                              'actuator_direction':'forward_direction',
#                                              'encoder_ticks':'actuator_torso_turn_encoder_ticks',
#                                              'actuator_limit_min':'actuator_torso_turn_limit_min',
#                                              'actuator_limit_max':'actuator_torso_turn_limit_max',
#                                              'actuator_encoder_centre':'actuator_torso_turn_encoder_centre',
#                                              'move_to_zero':'move_to_zero_true',
#                                              'get_one_limit_only':'get_one_limit_only_false',
#                                              'complete_calibration':'complete_calibration_true'},
#                                   transitions={'success':'torso_calibrated',
#                                                'error':'torso_calibration_failed'}),
#        smach.StateMachine.add('TorsoCalibration',
#                               sm_torso,
#                               transitions={'torso_calibrated':'ArmCalibration',
#                                            'torso_calibration_failed':'calibration_failed'})

        sm_arm = smach.StateMachine(outcomes=['arm_calibrated', 'arm_calibration_failed'])

        with sm_arm:

            sm_arm.userdata.forward_direction = 1
            sm_arm.userdata.backward_direction = -1
            sm_arm.userdata.get_one_limit_only_true = True
            sm_arm.userdata.get_one_limit_only_false = False
            sm_arm.userdata.complete_calibration_true = True
            sm_arm.userdata.complete_calibration_false = False
            sm_arm.userdata.encoder_goal_true = True
            sm_arm.userdata.encoder_goal_false = False
            sm_arm.userdata.move_to_hard_limit_true = True
            sm_arm.userdata.move_to_hard_limit_false = False
            sm_arm.userdata.move_to_zero_true = True
            sm_arm.userdata.move_to_zero_false = False

            sm_arm.userdata.actuator_torso_lift_name = "torso_lift"
            sm_arm.userdata.actuator_torso_lift_array_pos = 5
            sm_arm.userdata.actuator_torso_lift_max_current = 80
            sm_arm.userdata.actuator_torso_lift_speed = 0.15
            sm_arm.userdata.actuator_torso_lift_encoder_ticks = 28800

            sm_arm.userdata.actuator_shoulder_name = "shoulder"
            sm_arm.userdata.actuator_shoulder_array_pos = 4
            sm_arm.userdata.actuator_shoulder_max_current = 80
            sm_arm.userdata.actuator_shoulder_angle_ratio = 1.76086956522 # '(max - min) / max' with min = -70, max = 92
            sm_arm.userdata.actuator_shoulder_speed = 0.005
            sm_arm.userdata.actuator_shoulder_encoder_ticks = 336000
            sm_arm.userdata.actuator_shoulder_limit_min = 0
            sm_arm.userdata.actuator_shoulder_limit_max = 0
            sm_arm.userdata.actuator_shoulder_encoder_centre = 0
            sm_arm.userdata.actuator_shoulder_goal_pos = 0.0 # 90 / 360 * 336000 = 84,000

            sm_arm.userdata.actuator_elbow_name = "elbow"
            sm_arm.userdata.actuator_elbow_array_pos = 0
            sm_arm.userdata.actuator_elbow_max_current = 140
            sm_arm.userdata.actuator_elbow_angle_ratio = 3.23913043478 # '(max - min) / max' with min = -206 (-200), max = 92
            sm_arm.userdata.actuator_elbow_speed = 0.004
            sm_arm.userdata.actuator_elbow_low_speed = 0.001
            sm_arm.userdata.actuator_elbow_encoder_ticks = 268800
            sm_arm.userdata.actuator_elbow_limit_min = 0
            sm_arm.userdata.actuator_elbow_limit_max = 0
            sm_arm.userdata.actuator_elbow_encoder_centre = 0
            sm_arm.userdata.actuator_elbow_goal_pos = 1.90

            sm_arm.userdata.actuator_wrist_name = "wrist"
            sm_arm.userdata.actuator_wrist_array_pos = 9
            sm_arm.userdata.actuator_wrist_max_current = 200
            sm_arm.userdata.actuator_wrist_angle_ratio = 2 # min/max = -/+ 100
            sm_arm.userdata.actuator_wrist_speed = 0.01
            sm_arm.userdata.actuator_wrist_encoder_ticks = 417000
            sm_arm.userdata.actuator_wrist_limit_min = 0
            sm_arm.userdata.actuator_wrist_limit_max = 0
            sm_arm.userdata.actuator_wrist_encoder_centre = 0

            sm_arm.userdata.actuator_gripper_name = "gripper"
            sm_arm.userdata.actuator_gripper_array_pos = 1
            sm_arm.userdata.actuator_gripper_max_current = 140
            sm_arm.userdata.actuator_gripper_angle_ratio = 1.025641026 # min = -2 (changed from -2), max = 78
            sm_arm.userdata.actuator_gripper_speed = 0.01
            sm_arm.userdata.actuator_gripper_encoder_ticks = 873600
            sm_arm.userdata.actuator_gripper_limit_min = 0
            sm_arm.userdata.actuator_gripper_limit_max = 0
            sm_arm.userdata.actuator_gripper_encoder_centre = 0

            sm_arm.userdata.wait_0sec = 0
            sm_arm.userdata.wait_1sec = 1.0

            '''
            Get shoulder max and elbow min limits
            '''
            sm_shoulder_max_elbow_min = smach.Concurrence(outcomes=['success', 'error'],
                                                          default_outcome='error',
                                                          outcome_map={'success':
                                                                      {'GetShoulderMaxLimit':'success',
                                                                       'GetElbowMinLimit':'success'},
                                                                      'error':
                                                                      {'GetShoulderMaxLimit':'error',
                                                                       'GetElbowMinLimit':'error'}},
                                                          input_keys=['forward_direction',
                                                                      'backward_direction',
                                                                      'get_one_limit_only_true',
                                                                      'complete_calibration_false',
                                                                      'move_to_zero_false',
                                                                      'actuator_shoulder_name',
                                                                      'actuator_shoulder_array_pos',
                                                                      'actuator_shoulder_max_current',
                                                                      'actuator_shoulder_angle_ratio',
                                                                      'actuator_shoulder_speed',
                                                                      'actuator_shoulder_encoder_ticks',
                                                                      'actuator_shoulder_limit_min',
                                                                      'actuator_shoulder_limit_max',
                                                                      'actuator_elbow_name',
                                                                      'actuator_elbow_array_pos',
                                                                      'actuator_elbow_max_current',
                                                                      'actuator_elbow_angle_ratio',
                                                                      'actuator_elbow_speed',
                                                                      'actuator_elbow_encoder_ticks',
                                                                      'actuator_elbow_limit_min',
                                                                      'actuator_elbow_limit_max',
                                                                      'actuator_elbow_encoder_centre'],
                                                          output_keys=['actuator_elbow_limit_min',
                                                                       'actuator_elbow_limit_max',
                                                                       'actuator_shoulder_limit_min',
                                                                       'actuator_shoulder_limit_max'])
            with sm_shoulder_max_elbow_min:

                smach.Concurrence.add('GetShoulderMaxLimit',
                                      actuator.CalibrateActuator(),
                                      remapping={'actuator_name':'actuator_shoulder_name',
                                                 'actuator_array_pos':'actuator_shoulder_array_pos',
                                                 'actuator_max_current':'actuator_shoulder_max_current',
                                                 'actuator_angle_ratio':'actuator_shoulder_angle_ratio',
                                                 'actuator_speed':'actuator_shoulder_speed',
                                                 'actuator_direction':'forward_direction',
                                                 'encoder_ticks':'actuator_shoulder_encoder_ticks',
                                                 'actuator_limit_min':'actuator_shoulder_limit_min',
                                                 'actuator_limit_max':'actuator_shoulder_limit_max',
                                                 'actuator_encoder_centre':'actuator_shoulder_encoder_centre',
                                                 'move_to_zero':'move_to_zero_false',
                                                 'get_one_limit_only':'get_one_limit_only_true',
                                                 'complete_calibration':'complete_calibration_false'})

                smach.Concurrence.add('GetElbowMinLimit',
                                      actuator.CalibrateActuator(),
                                      remapping={'actuator_name':'actuator_elbow_name',
                                                 'actuator_array_pos':'actuator_elbow_array_pos',
                                                 'actuator_max_current':'actuator_elbow_max_current',
                                                 'actuator_angle_ratio':'actuator_elbow_angle_ratio',
                                                 'actuator_speed':'actuator_elbow_speed',
                                                 'actuator_direction':'backward_direction',
                                                 'encoder_ticks':'actuator_elbow_encoder_ticks',
                                                 'actuator_limit_min':'actuator_elbow_limit_min',
                                                 'actuator_limit_max':'actuator_elbow_limit_max',
                                                 'actuator_encoder_centre':'actuator_elbow_encoder_centre',
                                                 'move_to_zero':'move_to_zero_false',
                                                 'get_one_limit_only':'get_one_limit_only_true',
                                                 'complete_calibration':'complete_calibration_false'})

            smach.StateMachine.add('GetShoulderMaxAndElbowMin',
                                   sm_shoulder_max_elbow_min,
                                   transitions={'success':'MoveTorsoAndElbowUpAndCalibrateWristAndGripper',
                                                'error':'arm_calibration_failed'})

            '''
            Move torso and elbow up and calibrate wrist and gripper
            '''
            sm_to_el_up_cal_wr_gr = smach.Concurrence(outcomes=['done', 'error'],
                                                      default_outcome='error',
                                                      outcome_map={'done':
                                                                   {'MoveTorsoAndElbowUp':'moved',
                                                                    'WristAndGripperCalibration':'success'},
                                                                   'error':
                                                                   {'MoveTorsoAndElbowUp':'error',
                                                                    'WristAndGripperCalibration':'error'}},
                                                      input_keys=['forward_direction',
                                                                  'encoder_goal_false',
                                                                  'move_to_hard_limit_false',
                                                                  'actuator_torso_lift_name',
                                                                  'actuator_torso_lift_array_pos',
                                                                  'actuator_torso_lift_max_current',
                                                                  'actuator_torso_lift_speed',
                                                                  'actuator_torso_lift_encoder_ticks',
                                                                  'actuator_elbow_name',
                                                                  'actuator_elbow_array_pos',
                                                                  'actuator_elbow_max_current',
                                                                  'actuator_elbow_speed',
                                                                  'actuator_elbow_encoder_ticks',
                                                                  'move_to_zero_true',
                                                                  'get_one_limit_only_false',
                                                                  'complete_calibration_true',
                                                                  'actuator_wrist_name',
                                                                  'actuator_wrist_array_pos',
                                                                  'actuator_wrist_max_current',
                                                                  'actuator_wrist_angle_ratio',
                                                                  'actuator_wrist_speed',
                                                                  'actuator_wrist_encoder_ticks',
                                                                  'actuator_wrist_limit_min',
                                                                  'actuator_wrist_limit_max',
                                                                  'actuator_wrist_encoder_centre',
                                                                  'actuator_gripper_name',
                                                                  'actuator_gripper_array_pos',
                                                                  'actuator_gripper_max_current',
                                                                  'actuator_gripper_angle_ratio',
                                                                  'actuator_gripper_speed',
                                                                  'actuator_gripper_encoder_ticks',
                                                                  'actuator_gripper_limit_min',
                                                                  'actuator_gripper_limit_max',
                                                                  'actuator_gripper_encoder_centre',
                                                                  'wait_1sec'])

            sm_to_el_up_cal_wr_gr.userdata.actuator_torso_lift_goal_pos = 65
            sm_to_el_up_cal_wr_gr.userdata.actuator_elbow_goal_pos = 1.57

            with sm_to_el_up_cal_wr_gr:
                '''
                Move torso and elbow up
                '''
                sm_torso_elbow_up = smach.Concurrence(outcomes=['moved', 'error'],
                                                      default_outcome='error',
                                                      outcome_map={'moved':
                                                                   {'MoveTorsoLiftUp':'success',
                                                                    'MoveElbowUp':'success'},
                                                                   'error':
                                                                   {'MoveTorsoLiftUp':'error',
                                                                    'MoveElbowUp':'error'}},
                                                      input_keys=['forward_direction',
                                                                  'encoder_goal_false',
                                                                  'move_to_hard_limit_false',
                                                                  'actuator_torso_lift_name',
                                                                  'actuator_torso_lift_array_pos',
                                                                  'actuator_torso_lift_max_current',
                                                                  'actuator_torso_lift_speed',
                                                                  'actuator_torso_lift_encoder_ticks',
                                                                  'actuator_elbow_name',
                                                                  'actuator_elbow_array_pos',
                                                                  'actuator_elbow_max_current',
                                                                  'actuator_elbow_speed',
                                                                  'actuator_elbow_encoder_ticks'])

                sm_torso_elbow_up.userdata.actuator_torso_lift_goal_pos = 65
                sm_torso_elbow_up.userdata.actuator_elbow_goal_pos = 1.57

                with sm_torso_elbow_up:
                    smach.Concurrence.add('MoveTorsoLiftUp',
                                          actuator.MoveActuator(),
                                          remapping={'actuator_name':'actuator_torso_lift_name',
                                                     'actuator_array_pos':'actuator_torso_lift_array_pos',
                                                     'actuator_max_current':'actuator_torso_lift_max_current',
                                                     'actuator_speed':'actuator_torso_lift_speed',
                                                     'actuator_direction':'forward_direction',
                                                     'encoder_ticks':'actuator_torso_lift_encoder_ticks',
                                                     'actuator_goal_pos':'actuator_torso_lift_goal_pos',
                                                     'encoder_goal':'encoder_goal_false',
                                                     'move_to_hard_limit':'move_to_hard_limit_false'})

                    smach.Concurrence.add('MoveElbowUp',
                                          actuator.MoveActuator(),
                                          remapping={'actuator_name':'actuator_elbow_name',
                                                     'actuator_array_pos':'actuator_elbow_array_pos',
                                                     'actuator_max_current':'actuator_elbow_max_current',
                                                     'actuator_speed':'actuator_elbow_speed',
                                                     'actuator_direction':'forward_direction',
                                                     'encoder_ticks':'actuator_elbow_encoder_ticks',
                                                     'actuator_goal_pos':'actuator_elbow_goal_pos',
                                                     'encoder_goal':'encoder_goal_false',
                                                     'move_to_hard_limit':'move_to_hard_limit_false'})

                smach.Concurrence.add('MoveTorsoAndElbowUp', sm_torso_elbow_up)

                '''
                Wrist and gripper calibration
                '''
                sm_wrist_gripper_calibration = smach.StateMachine(outcomes=['success',
                                                                             'error'],
                                                                   input_keys=['forward_direction',
                                                                               'move_to_zero_true',
                                                                               'get_one_limit_only_false',
                                                                               'complete_calibration_true',
                                                                               'actuator_wrist_name',
                                                                               'actuator_wrist_array_pos',
                                                                               'actuator_wrist_max_current',
                                                                               'actuator_wrist_angle_ratio',
                                                                               'actuator_wrist_speed',
                                                                               'actuator_wrist_encoder_ticks',
                                                                               'actuator_wrist_limit_min',
                                                                               'actuator_wrist_limit_max',
                                                                               'actuator_wrist_encoder_centre',
                                                                               'actuator_gripper_name',
                                                                               'actuator_gripper_array_pos',
                                                                               'actuator_gripper_max_current',
                                                                               'actuator_gripper_angle_ratio',
                                                                               'actuator_gripper_speed',
                                                                               'actuator_gripper_encoder_ticks',
                                                                               'actuator_gripper_limit_min',
                                                                               'actuator_gripper_limit_max',
                                                                               'actuator_gripper_encoder_centre',
                                                                               'wait_1sec'])

                with sm_wrist_gripper_calibration:

                    smach.StateMachine.add('WaitBeforeWristAndGripperCalibration',
                               misc_tools.Wait(),
                               remapping={'duration':'wait_1sec'},
                               transitions={'done':'Calibration'})

                    sm_calibration = smach.Concurrence(outcomes=['calibrated',
                                                                 'calibration_failed'],
                                                       default_outcome='calibration_failed',
                                                       outcome_map={'calibrated':
                                                                    {'CalibrateWrist':'success',
                                                                     'CalibrateGripper':'success'},
                                                                     'calibration_failed':
                                                                    {'CalibrateWrist':'error',
                                                                     'CalibrateGripper':'error'}},
                                                       input_keys=['forward_direction',
                                                                   'move_to_zero_true',
                                                                   'get_one_limit_only_false',
                                                                   'complete_calibration_true',
                                                                   'actuator_wrist_name',
                                                                   'actuator_wrist_array_pos',
                                                                   'actuator_wrist_max_current',
                                                                   'actuator_wrist_angle_ratio',
                                                                   'actuator_wrist_speed',
                                                                   'actuator_wrist_encoder_ticks',
                                                                   'actuator_wrist_limit_min',
                                                                   'actuator_wrist_limit_max',
                                                                   'actuator_wrist_encoder_centre',
                                                                   'actuator_gripper_name',
                                                                   'actuator_gripper_array_pos',
                                                                   'actuator_gripper_max_current',
                                                                   'actuator_gripper_angle_ratio',
                                                                   'actuator_gripper_speed',
                                                                   'actuator_gripper_encoder_ticks',
                                                                   'actuator_gripper_limit_min',
                                                                   'actuator_gripper_limit_max',
                                                                   'actuator_gripper_encoder_centre'])
                    with sm_calibration:
                        smach.Concurrence.add('CalibrateWrist',
                                              actuator.CalibrateActuator(),
                                              remapping={'actuator_name':'actuator_wrist_name',
                                                         'actuator_array_pos':'actuator_wrist_array_pos',
                                                         'actuator_max_current':'actuator_wrist_max_current',
                                                         'actuator_angle_ratio':'actuator_wrist_angle_ratio',
                                                         'actuator_speed':'actuator_wrist_speed',
                                                         'actuator_direction':'forward_direction',
                                                         'encoder_ticks':'actuator_wrist_encoder_ticks',
                                                         'actuator_limit_min':'actuator_wrist_limit_min',
                                                         'actuator_limit_max':'actuator_wrist_limit_max',
                                                         'actuator_encoder_centre':'actuator_wrist_encoder_centre',
                                                         'move_to_zero':'move_to_zero_true',
                                                         'get_one_limit_only':'get_one_limit_only_false',
                                                         'complete_calibration':'complete_calibration_true'})

                        smach.Concurrence.add('CalibrateGripper',
                                              actuator.CalibrateActuator(),
                                              remapping={'actuator_name':'actuator_gripper_name',
                                                         'actuator_array_pos':'actuator_gripper_array_pos',
                                                         'actuator_max_current':'actuator_gripper_max_current',
                                                         'actuator_angle_ratio':'actuator_gripper_angle_ratio',
                                                         'actuator_speed':'actuator_gripper_speed',
                                                         'actuator_direction':'forward_direction',
                                                         'encoder_ticks':'actuator_gripper_encoder_ticks',
                                                         'actuator_limit_min':'actuator_gripper_limit_min',
                                                         'actuator_limit_max':'actuator_gripper_limit_max',
                                                         'actuator_encoder_centre':'actuator_gripper_encoder_centre',
                                                         'move_to_zero':'move_to_zero_true',
                                                         'get_one_limit_only':'get_one_limit_only_false',
                                                         'complete_calibration':'complete_calibration_true'})

                    smach.StateMachine.add('Calibration',
                                           sm_calibration,
#                                           remapping={'forward_direction':'forward_direction',
#                                                      'move_to_zero_true':'move_to_zero_true',
#                                                      'get_one_limit_only_false':'get_one_limit_only_false',
#                                                      'complete_calibration_true':'complete_calibration_true',
#                                                      'actuator_wrist_name':'actuator_wrist_name',
#                                                      'actuator_wrist_array_pos':'actuator_wrist_array_pos',
#                                                      'actuator_wrist_max_current':'actuator_wrist_max_current',
#                                                      'actuator_wrist_angle_ratio':'actuator_wrist_angle_ratio',
#                                                      'actuator_wrist_speed':'actuator_wrist_speed',
#                                                      'actuator_wrist_encoder_ticks':'actuator_wrist_encoder_ticks',
#                                                      'actuator_wrist_limit_min':'actuator_wrist_limit_min',
#                                                      'actuator_wrist_limit_max':'actuator_wrist_limit_max',
#                                                      'actuator_wrist_encoder_centre':'actuator_wrist_encoder_centre',
#                                                      'actuator_gripper_name':'actuator_gripper_name',
#                                                      'actuator_gripper_array_pos':'actuator_gripper_array_pos',
#                                                      'actuator_gripper_max_current':'actuator_gripper_max_current',
#                                                      'actuator_gripper_angle_ratio':'actuator_gripper_angle_ratio',
#                                                      'actuator_gripper_speed':'actuator_gripper_speed',
#                                                      'actuator_gripper_encoder_ticks':'actuator_gripper_encoder_ticks',
#                                                      'actuator_gripper_limit_min':'actuator_gripper_limit_min',
#                                                      'actuator_gripper_limit_max':'actuator_gripper_limit_max',
#                                                      'actuator_gripper_encoder_centre':'actuator_gripper_encoder_centre'},
                                        transitions={'calibrated':'success',
                                                     'calibration_failed':'error'})

                smach.Concurrence.add('WristAndGripperCalibration', sm_wrist_gripper_calibration)

            smach.StateMachine.add('MoveTorsoAndElbowUpAndCalibrateWristAndGripper',
                                   sm_to_el_up_cal_wr_gr,
                                   transitions={'done':'MoveShoulderHardMin',
                                                'error':'arm_calibration_failed'})

            '''
            TODO:
                try to add stay at hard limit to CalibrateActuator
                then combine the below with FinishShoulderCalibration
            '''
            smach.StateMachine.add('MoveShoulderHardMin',
                                   actuator.MoveActuator(),
                                   transitions={'success':'FinishElbowCalibration',
                                                'error':'arm_calibration_failed'},
                                   remapping={'actuator_name':'actuator_shoulder_name',
                                              'actuator_array_pos':'actuator_shoulder_array_pos',
                                              'actuator_max_current':'actuator_shoulder_max_current',
                                              'actuator_speed':'actuator_shoulder_speed',
                                              'actuator_direction':'backward_direction',
                                              'encoder_ticks':'actuator_shoulder_encoder_ticks',
                                              'actuator_goal_pos':'actuator_shoulder_goal_pos',
                                              'encoder_goal':'encoder_goal_true',
                                              'move_to_hard_limit':'move_to_hard_limit_true'})

            smach.StateMachine.add('FinishElbowCalibration',
                                   actuator.CalibrateActuator(),
                                   transitions={'success':'MoveElbowAgain',
                                                'error':'arm_calibration_failed'},
                                   remapping={'actuator_name':'actuator_elbow_name',
                                              'actuator_array_pos':'actuator_elbow_array_pos',
                                              'actuator_max_current':'actuator_elbow_max_current',
                                              'actuator_angle_ratio':'actuator_elbow_angle_ratio',
                                              'actuator_speed':'actuator_elbow_speed',
                                              'actuator_direction':'forward_direction',
                                              'encoder_ticks':'actuator_elbow_encoder_ticks',
                                              'actuator_limit_min':'actuator_elbow_limit_min',
                                              'actuator_limit_max':'actuator_elbow_limit_max',
                                              'actuator_encoder_centre':'actuator_elbow_encoder_centre',
                                              'move_to_zero':'move_to_zero_false',
                                              'get_one_limit_only':'get_one_limit_only_true',
                                              'complete_calibration':'complete_calibration_true'})

            smach.StateMachine.add('MoveElbowAgain',
                                   actuator.MoveActuator(),
                                   transitions={'success':'FinishShoulderCalibration',
                                                'error':'arm_calibration_failed'},
                                   remapping={'actuator_name':'actuator_elbow_name',
                                              'actuator_array_pos':'actuator_elbow_array_pos',
                                              'actuator_max_current':'actuator_elbow_max_current',
                                              'actuator_speed':'actuator_elbow_speed',
                                              'actuator_direction':'backward_direction',
                                              'encoder_ticks':'actuator_elbow_encoder_ticks',
                                              'actuator_goal_pos':'actuator_elbow_goal_pos',
                                              'encoder_goal':'encoder_goal_false',
                                              'move_to_hard_limit':'move_to_hard_limit_false'})

            smach.StateMachine.add('FinishShoulderCalibration',
                                   actuator.CalibrateActuator(),
                                   transitions={'success':'MoveTorsoDownAndElbowToZero',
                                                'error':'arm_calibration_failed'},
                                   remapping={'actuator_name':'actuator_shoulder_name',
                                              'actuator_array_pos':'actuator_shoulder_array_pos',
                                              'actuator_max_current':'actuator_shoulder_max_current',
                                              'actuator_angle_ratio':'actuator_shoulder_angle_ratio',
                                              'actuator_speed':'actuator_shoulder_speed',
                                              'actuator_direction':'backward_direction',
                                              'encoder_ticks':'actuator_shoulder_encoder_ticks',
                                              'actuator_limit_min':'actuator_shoulder_limit_min',
                                              'actuator_limit_max':'actuator_shoulder_limit_max',
                                              'actuator_encoder_centre':'actuator_shoulder_encoder_centre',
                                              'move_to_zero':'move_to_zero_true',
                                              'get_one_limit_only':'get_one_limit_only_true',
                                              'complete_calibration':'complete_calibration_true'})
            '''
            Move torso down and elbow to zero
            '''
            sm_torso_down_elbow_zero = smach.Concurrence(outcomes=['moved', 'error'],
                                                         default_outcome='error',
                                                         outcome_map={'moved':
                                                                      {'MoveElbowToZero':'success',
                                                                       'MoveTorsoLiftDown':'success'},
                                                                      'error':
                                                                      {'MoveElbowToZero':'error',
                                                                       'MoveTorsoLiftDown':'error'}},
                                                         input_keys=['forward_direction',
                                                                     'backward_direction',
                                                                     'encoder_goal_true',
                                                                     'encoder_goal_false',
                                                                     'move_to_hard_limit_false',
                                                                     'actuator_elbow_name',
                                                                     'actuator_elbow_array_pos',
                                                                     'actuator_elbow_max_current',
                                                                     'actuator_elbow_speed',
                                                                     'actuator_elbow_encoder_ticks',
                                                                     'actuator_elbow_encoder_centre',
                                                                     'actuator_torso_lift_name',
                                                                     'actuator_torso_lift_array_pos',
                                                                     'actuator_torso_lift_max_current',
                                                                     'actuator_torso_lift_speed',
                                                                     'actuator_torso_lift_encoder_ticks', ])

            sm_torso_down_elbow_zero.userdata.actuator_torso_lift_goal_pos = 0.0
            with sm_torso_down_elbow_zero:
                smach.Concurrence.add('MoveElbowToZero',
                                      actuator.MoveActuator(),
                                      remapping={'actuator_name':'actuator_elbow_name',
                                                 'actuator_array_pos':'actuator_elbow_array_pos',
                                                 'actuator_max_current':'actuator_elbow_max_current',
                                                 'actuator_speed':'actuator_elbow_speed',
                                                 'actuator_direction':'backward_direction',
                                                 'actuator_goal_pos':'actuator_elbow_encoder_centre',
                                                 'encoder_ticks':'actuator_elbow_encoder_ticks',
                                                 'encoder_goal':'encoder_goal_true',
                                                 'move_to_hard_limit':'move_to_hard_limit_false'})

                smach.Concurrence.add('MoveTorsoLiftDown',
                                      actuator.MoveActuator(),
                                      remapping={'actuator_name':'actuator_torso_lift_name',
                                                 'actuator_array_pos':'actuator_torso_lift_array_pos',
                                                 'actuator_max_current':'actuator_torso_lift_max_current',
                                                 'actuator_speed':'actuator_torso_lift_speed',
                                                 'actuator_direction':'backward_direction',
                                                 'actuator_goal_pos':'actuator_torso_lift_goal_pos',
                                                 'encoder_ticks':'actuator_torso_lift_encoder_ticks',
                                                 'encoder_goal':'encoder_goal_false',
                                                 'move_to_hard_limit':'move_to_hard_limit_false'})

            smach.StateMachine.add('MoveTorsoDownAndElbowToZero',
                                   sm_torso_down_elbow_zero,
                                   transitions={'moved':'ResetEncoderTorsoTurn',
                                                'error':'arm_calibration_failed'})

            smach.StateMachine.add('ResetEncoderTorsoTurn',
                                   ServiceState('goo/reset_encoders',
                                                goo_srvs.ResetEncoders,
                                                request=goo_srvs.ResetEncodersRequest("torso_turn")),
                                                transitions={'succeeded':'ResetEncoderShoulder',
                                                             'preempted':'arm_calibration_failed',
                                                             'aborted':'arm_calibration_failed', })
            smach.StateMachine.add('ResetEncoderShoulder',
                                   ServiceState('goo/reset_encoders',
                                                goo_srvs.ResetEncoders,
                                                request=goo_srvs.ResetEncodersRequest("shoulder")),
                                                transitions={'succeeded':'ResetEncoderElbow',
                                                             'preempted':'arm_calibration_failed',
                                                             'aborted':'arm_calibration_failed', })

            smach.StateMachine.add('ResetEncoderElbow',
                                   ServiceState('goo/reset_encoders',
                                                goo_srvs.ResetEncoders,
                                                request=goo_srvs.ResetEncodersRequest("elbow")),
                                                transitions={'succeeded':'ResetEncoderWrist',
                                                             'preempted':'arm_calibration_failed',
                                                             'aborted':'arm_calibration_failed', })

            smach.StateMachine.add('ResetEncoderWrist',
                                   ServiceState('goo/reset_encoders',
                                                goo_srvs.ResetEncoders,
                                                request=goo_srvs.ResetEncodersRequest("wrist")),
                                                transitions={'succeeded':'ResetEncoderGripper',
                                                             'preempted':'arm_calibration_failed',
                                                             'aborted':'arm_calibration_failed', })

            smach.StateMachine.add('ResetEncoderGripper',
                                   ServiceState('goo/reset_encoders',
                                                goo_srvs.ResetEncoders,
                                                request=goo_srvs.ResetEncodersRequest("gripper")),
                                                transitions={'succeeded':'arm_calibrated',
                                                             'preempted':'arm_calibration_failed',
                                                             'aborted':'arm_calibration_failed', })

        smach.StateMachine.add('ArmCalibration', sm_arm,
                               transitions={'arm_calibrated':'WaitBeforeMoveArmToDefaultPose',
                                            'arm_calibration_failed':'calibration_failed'})

        smach.StateMachine.add('WaitBeforeMoveArmToDefaultPose',
                               misc_tools.Wait(),
                               remapping={'duration':'wait_2sec'},
                               transitions={'done':'MoveArmToDefaultPose'})

        trajectory = trajectory_msgs.JointTrajectory()
        trajectory.joint_names = ["torso_turn", "torso_lift", "shoulder", "elbow", "wrist"]
        waypoint = trajectory_msgs.JointTrajectoryPoint()
#        waypoint.positions = [0.0, 0.0, 1.57, -3.14, 1.57] when well calibrated
        waypoint.positions = [0.0, 0.0, 1.56, -3.13, 1.57] # workaround after PID tuning
        waypoint.velocities = [0.3] * len(trajectory.joint_names)
        waypoint.accelerations = [0.0] * len(trajectory.joint_names)
        trajectory.points.append(waypoint)
        sm_arm_calibration.userdata.arm_default_pose_trajectory = trajectory

        smach.StateMachine.add('MoveArmToDefaultPose',
                               SimpleActionState('arm_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_slots=['trajectory']),
                               remapping={'trajectory':'arm_default_pose_trajectory'},
                               transitions={'succeeded':'calibrated',
                                            'aborted':'calibration_failed',
                                            'preempted':'calibration_failed'})

    return sm_arm_calibration
