#!/usr/bin/env python
import roslib; roslib.load_manifest('thorp_smach')
import rospy
import smach
import device_msgs.msg as device_msgs
import goo_msgs.msg as goo_msgs


class CalibrateActuator(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success','error'],
                             input_keys=['actuator_name',
                                         'actuator_array_pos',
                                         'actuator_max_current',
                                         'actuator_angle_ratio',
                                         'actuator_speed',
                                         'actuator_direction',
                                         'encoder_ticks',
                                         'actuator_limit_min',
                                         'actuator_limit_max',
                                         'actuator_encoder_centre',
                                         'move_to_zero',
                                         'get_one_limit_only',
                                         'complete_calibration'],
                             output_keys=['actuator_limit_min',
                                          'actuator_limit_max',
                                          'actuator_encoder_centre'])
        self.actuator_name = ""
        self.actuator_array_pos = 0
        self.actuator_max_current = 0
        self.actuator_command = device_msgs.ActuatorCommand()
        self.actuator_command.name = self.actuator_name
        self.actuator_command.position = 0.0
        self.actuator_command.velocity = 0.0
        self.actuator_command.enabled = device_msgs.ActuatorCommand.no_change
        self.actuator_command_ready = False
        self.actuator_reached_limit = False
        self.current_actuator_pos = 0.0
        self.encoder_current = 0.0
    
    def actuator_state_cb(self, data):
        self.current_actuator_pos = data.angle
        if self.actuator_command_ready is False:
            self.actuator_command.position = self.current_actuator_pos
            self.actuator_command_ready = True
            rospy.loginfo("Command for actuator '" + self.actuator_name + "' initialised (" 
                          + str(self.current_actuator_pos) + ") [" + self.actuator_name + "]")
    
    def goo_state_cb(self, data):
        self.encoder_current = data.encoders[self.actuator_array_pos]
        current = data.currents[self.actuator_array_pos]
        if current > self.actuator_max_current:
            if self.actuator_reached_limit is False:
                rospy.loginfo("Actuator '" + self.actuator_name + "' reached a limit! (current = " + str(current) 
                              + ") [" + self.actuator_name + "]")
            self.actuator_reached_limit = True
        elif current < self.actuator_max_current * 0.75:
            if self.actuator_reached_limit is True:
                 rospy.loginfo("Actuator '" + self.actuator_name + "' left the limit. [" + self.actuator_name + "]")
            self.actuator_reached_limit = False

    def execute(self, userdata):
        self.actuator_name = userdata.actuator_name
        self.actuator_array_pos = userdata.actuator_array_pos
        self.actuator_max_current = userdata.actuator_max_current
        self.actuator_command.name = self.actuator_name
        step = userdata.actuator_speed
        direction = userdata.actuator_direction
        limit_max = userdata.actuator_limit_max
        limit_min = userdata.actuator_limit_min
        angle_ratio = userdata.actuator_angle_ratio
        ticks = userdata.encoder_ticks
        rospy.loginfo("Preparing calibration for actuator '" + self.actuator_name + "' with:")
        rospy.loginfo('    array position: ' + str(self.actuator_array_pos))
        rospy.loginfo('    max current: ' + str(self.actuator_max_current))
        rospy.loginfo('    actuator angle ratio: ' + str(angle_ratio))
        rospy.loginfo('    actuator speed: ' + str(step))
        rospy.loginfo('    actuator direction: ' + str(direction))
        rospy.loginfo('    actuator limit min: ' + str(limit_min))
        rospy.loginfo('    actuator limit max: ' + str(limit_max))
        rospy.loginfo('    encoder ticks: ' + str(userdata.encoder_ticks))
        rospy.loginfo('    move to zero: ' + str(userdata.move_to_zero))
        rospy.loginfo('    get one limit only: ' + str(userdata.get_one_limit_only))
        rospy.loginfo('    complete calibration: ' + str(userdata.complete_calibration))
        pub_actuator_command = rospy.Publisher("actuator_command/" + self.actuator_name,
                                               device_msgs.ActuatorCommand)
        sub_actuator_state = rospy.Subscriber("actuator_state/" + self.actuator_name,
                                              device_msgs.ActuatorState,
                                              self.actuator_state_cb)
        sub_goo_state = rospy.Subscriber("goo/state", goo_msgs.State, self.goo_state_cb)
        encs = 0
        encoder_sum = 0
        encoder_limit = 0
        encoder_center = 0
        abs_current = 0
        
        # Parameters for moving
        max_dist = userdata.encoder_ticks / 36 # 10 deg
        min_dist = userdata.encoder_ticks / 360 # 1 deg
        target_dist = userdata.encoder_ticks / 7200 # 0.05 deg
        max_vel_fac = 1.0 # velocity factor at max_dist
        min_vel_fac = 0.1 #  velocity factor at min_dist
        min_vel = 0.000436332312999 # 0.025deg/s,
        dist = 0
        last_dist = 0
        
        encoder_limit_min_set = False
        encoder_limit_max_set = False
        move_out_of_limit = False
        aggregating = False
        spin_rate = rospy.Rate(20)
        if not(userdata.get_one_limit_only or userdata.complete_calibration):
            rospy.loginfo(">>>>>>>>>> Strange, apparently there is nothing to do. Cool! [" + self.actuator_name + "]")
            return 'success'
        elif (userdata.get_one_limit_only and userdata.complete_calibration):
            if ((userdata.actuator_limit_max is 0) and (userdata.actuator_limit_min is 0)):
                rospy.logerr("Can't complete calibration, because one limit hasn't been set! [" 
                             + self.actuator_name + "]")
                return 'error'
            else:
                if not(userdata.actuator_limit_max is 0):
                    encoder_limit_max_set = True
                    rospy.loginfo("Max limit set. Will determine min limit. [" + self.actuator_name + "]")
                else:
                    encoder_limit_min_set = True
                    rospy.loginfo("Min limit set. Will determine max limit. [" + self.actuator_name + "]")
        rospy.loginfo("Starting calibration ... [" + self.actuator_name + "]")
        while not rospy.is_shutdown():
            if self.actuator_command_ready:
                #=======================================================================================================
                # Calculating
                #=======================================================================================================
                if self.actuator_reached_limit or aggregating:
                    if move_out_of_limit:
                        rospy.logdebug("Moving out of limit ...")
                        self.actuator_command.position = self.actuator_command.position + step * direction
                        pub_actuator_command.publish(self.actuator_command)
                    elif (encs < 10):
                        aggregating = True
                        encoder_sum = encoder_sum + self.encoder_current
                        encs = encs + 1
                        if (abs(self.actuator_command.position - self.current_actuator_pos) > 0.01):
                            self.actuator_command.position = self.current_actuator_pos
                            pub_actuator_command.publish(self.actuator_command)
                            rospy.loginfo("Commanded position set to current position (" 
                                          + str(self.current_actuator_pos) + "). [" + self.actuator_name + "]")
                    else:
                        encs = 0
                        aggregating = False
                        encoder_limit = int(encoder_sum / 10)
                        encoder_sum = 0
                        if direction > 0:
                            rospy.loginfo(">>>>>>>>>> New aggregated encoder value @ max limit: " 
                                          + str(encoder_limit) + " [" + self.actuator_name + "]")
                            userdata.actuator_limit_max = encoder_limit
                            encoder_limit_max_set = True
                        else:
                            rospy.loginfo(">>>>>>>>>> New aggregated encoder value @ min limit: " 
                                          + str(encoder_limit) + " [" + self.actuator_name + "]")
                            userdata.actuator_limit_min = encoder_limit
                            encoder_limit_min_set = True
                        direction = -direction
                        move_out_of_limit = True
                #=======================================================================================================
                # Moving
                #=======================================================================================================
                else:
                    move_out_of_limit = False
                    if (userdata.get_one_limit_only and not(userdata.complete_calibration)):
                        if (encoder_limit_min_set or encoder_limit_max_set):
                            rospy.loginfo(">>>>>>>>>> Finished getting one limit! <<<<<<<<<< [" 
                                          + self.actuator_name + "]")
                            return 'success'
                    else:
                        if (encoder_limit_min_set and encoder_limit_max_set):
                            if encoder_center is 0:
                                limit_max = userdata.actuator_limit_max
                                limit_min = userdata.actuator_limit_min
                                angle_ratio = userdata.actuator_angle_ratio
                                encoder_center = limit_max - ((limit_max - limit_min) / angle_ratio)
                                rospy.loginfo(">>>>>>>>>> Encoder value @ centre = " + str(encoder_center)
                                              + " [" + self.actuator_name + "]")
                                userdata.actuator_encoder_centre = encoder_center
                                last_dist = abs(encoder_center - self.encoder_current)
                            else:
                                if (userdata.move_to_zero):
                                    diff = encoder_center - self.encoder_current
                                    dist = abs(diff)
                                    if dist > last_dist:
                                        rospy.loginfo("Overshoot detected at " + str(dist) 
                                                      + " (" + str(diff) + "). [" + self.actuator_name + "]")
                                        direction = -direction
                                        rospy.loginfo("Direction inverted (now " + str(direction) 
                                                      + "). [" + self.actuator_name + "]")
                                        if userdata.actuator_speed * min_vel_fac > min_vel:
                                            min_vel_fac = min_vel_fac * 0.1
                                            rospy.loginfo("Min speed reduced (now " 
                                                          + str(userdata.actuator_speed * min_vel_fac)
                                                          + "). [" + self.actuator_name + "]")
                                    last_dist = dist
                                    if (dist <= target_dist):
                                        rospy.loginfo(">>>>>>>>>> Stopped at new centre. [" + self.actuator_name + "]")
                                        rospy.loginfo("centre position = " + str(encoder_center) + " [" + self.actuator_name + "]")
                                        rospy.loginfo("current position = " + str(self.encoder_current)+ " [" + self.actuator_name + "]")
                                        userdata.actuator_encoder_centre = encoder_center
                                        return 'success'
                                    elif (dist < max_dist) and (dist > min_dist):
                                        m = (min_vel_fac - max_vel_fac) / (max_dist - min_dist)
                                        step = userdata.actuator_speed * (m * (max_dist - dist) + max_vel_fac)
                                        rospy.loginfo("Reducing velocity (vel = " + str(step) + ", dist = " + str(dist)
                                                      + ", diff = " + str(diff) + ") ... [" + self.actuator_name + "]")
                                    elif (dist <= min_dist):
                                        if step > min_vel_fac * userdata.actuator_speed:
                                            step = min_vel_fac * userdata.actuator_speed
                                            rospy.loginfo("Moving with minimum velocity ... (" + str(dist) + ") [" + self.actuator_name + "]")
                                else:
                                    rospy.loginfo("'Move to zero position' not requested. Finishing.")
                                    return 'success'
                    self.actuator_command.position = self.actuator_command.position + step * direction
                    pub_actuator_command.publish(self.actuator_command)
            spin_rate.sleep()
        return 'error'

class MoveActuator(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success','error'],
                             input_keys=['actuator_name',
                                         'actuator_array_pos',
                                         'actuator_max_current',
                                         'actuator_speed',
                                         'actuator_direction',
                                         'encoder_ticks',
                                         'actuator_goal_pos',
                                         'encoder_goal',
                                         'move_to_hard_limit'])
        self.actuator_name = ""
        self.actuator_array_pos = 0
        self.actuator_max_current = 0
        self.actuator_angle_ratio = 0.0
        self.actuator_speed = 0.0
        self.actuator_direction = 1
        self.actuator_goal_pos = 0.0
        self.actuator_command = device_msgs.ActuatorCommand()
        self.actuator_command.name = self.actuator_name
        self.actuator_command.position = 0.0
        self.actuator_command.velocity = 0.0
        self.actuator_command.enabled = device_msgs.ActuatorCommand.no_change
        self.current_actuator_pos = 0.0
        self.current_encoder_pos = 0.0
        self.actuator_ready = False
        self.encoder_ready = False
        self.actuator_reached_limit = False
    
    def actuator_state_cb(self, data):
        self.current_actuator_pos = data.angle
        if self.actuator_ready is False:
            self.actuator_command.position = self.current_actuator_pos
            self.actuator_ready = True
            rospy.loginfo("Actuator initialised (" + str(self.current_actuator_pos) + ") [" + self.actuator_name + "]")
            
    def goo_state_cb(self, data):
        self.current_encoder_pos = data.encoders[self.actuator_array_pos]
        if self.encoder_ready is False:
            self.encoder_ready = True
            rospy.loginfo("Encoder initialised (" + str(self.current_encoder_pos) + ") [" + self.actuator_name + "]")
        current = data.currents[self.actuator_array_pos]
        if current > self.actuator_max_current:
            if self.actuator_reached_limit is False:
                rospy.loginfo("Actuator reached a limit! (current = " + str(current) + ") [" + self.actuator_name + "]")
            self.actuator_reached_limit = True
        elif current < self.actuator_max_current * 0.75:
            if self.actuator_reached_limit is True:
                 rospy.loginfo("Actuator left the limit. [" + self.actuator_name + "]")
            self.actuator_reached_limit = False

    def execute(self, userdata):
        self.actuator_name = userdata.actuator_name
        self.actuator_array_pos = userdata.actuator_array_pos
        self.actuator_speed = userdata.actuator_speed
        self.actuator_direction = userdata.actuator_direction
        self.actuator_goal_pos = userdata.actuator_goal_pos
        self.actuator_max_current = userdata.actuator_max_current
        self.actuator_command.name = self.actuator_name
        rospy.loginfo("Preparing pubs/subs for actuator '" + self.actuator_name + "' with:")
        rospy.loginfo('array position: ' + str(self.actuator_array_pos))
        rospy.loginfo('max current: ' + str(self.actuator_max_current))
        rospy.loginfo('actuator speed: ' + str(self.actuator_speed))
        rospy.loginfo('actuator direction: ' + str(self.actuator_direction))
        rospy.loginfo('goal position: ' + str(self.actuator_goal_pos))
        rospy.loginfo('move to encoder goal: ' + str(userdata.encoder_goal))
        rospy.loginfo('move to hard limit: ' + str(userdata.move_to_hard_limit))
        pub_actuator_command = rospy.Publisher("actuator_command/" + self.actuator_name,
                                               device_msgs.ActuatorCommand)
        sub_actuator_state = rospy.Subscriber("actuator_state/" + self.actuator_name,
                                              device_msgs.ActuatorState,
                                              self.actuator_state_cb)
        if userdata.encoder_goal:
            sub_goo_state = rospy.Subscriber("goo/state",
                                             goo_msgs.State,
                                             self.goo_state_cb)
        else:
            self.encoder_ready = True
        if userdata.move_to_hard_limit:
            step = self.actuator_speed
        else:
            step = 2 * self.actuator_speed  # moving freely should not run into a limit, so fast speed is ok
        direction = self.actuator_direction
        
        # Parameters for moving
        target_dist = 0.0
        dist = 0.0
        last_dist = 0.0
        max_dist = 0.0
        min_dist = 0.0
        max_vel_fac = 1.0 # velocity factor at max_dist
        min_vel_fac = 0.1 #  velocity factor at min_dist
        min_vel = 0.000436332312999 # 0.025deg/s,
        multi = 1.0
        if userdata.encoder_goal:
            max_dist = userdata.encoder_ticks / 36 # 10 deg
            min_dist = userdata.encoder_ticks / 360 # 2 deg
            target_dist = userdata.encoder_ticks / 7200 # 0.05 deg
        else:
            if self.actuator_name is 'torso_lift':
                multi = 10.0
                rospy.loginfo("Setting speed limits for 'torso_lift'.")
            max_dist = 0.174532925199 * multi # 10 deg
            min_dist = 0.0349065850399 * multi # 2 deg
            target_dist = 0.0174532925199 * multi # 1 deg
        
        spin_rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if (self.actuator_ready and self.encoder_ready):
                break
            else:
                rospy.loginfo("Waiting for initialisation ... [" + self.actuator_name + "]")
                rospy.sleep(0.1)
        if userdata.encoder_goal:
            last_dist = abs(self.actuator_goal_pos - self.current_encoder_pos)
        else:
            last_dist = abs(self.actuator_goal_pos - self.current_actuator_pos)
        rospy.loginfo('Starting to move ...')
        while not rospy.is_shutdown():
            if (self.actuator_reached_limit):
                if (abs(self.actuator_command.position - self.current_actuator_pos) > 0.02):
                    self.actuator_command.position = self.current_actuator_pos
                    pub_actuator_command.publish(self.actuator_command)
                    rospy.loginfo("Commanded position set to current position (" 
                                  + str(self.current_actuator_pos) + ").")
                if (userdata.move_to_hard_limit):
                    rospy.loginfo(">>>>>>>>>> Stopped at target hard limit.")
                    return 'success'
                else:
                    rospy.logerr(">>>>>>>>>> Stopped moving due to reaching an hard limit!")
                    return 'error'
            else:
                if not(userdata.move_to_hard_limit):
                    if userdata.encoder_goal:
                        dist = abs(self.actuator_goal_pos - self.current_encoder_pos)
                    else:
                        dist = abs(self.actuator_goal_pos - self.current_actuator_pos)
                    if dist > last_dist:
                        rospy.loginfo("Overshoot detected at " + str(dist) 
                                      + ". [" + self.actuator_name + "]")
                        direction = -direction
                        rospy.loginfo("Direction inverted (now " + str(direction) 
                                      + "). [" + self.actuator_name + "]")
                        if userdata.actuator_speed * min_vel_fac > min_vel:
                            min_vel_fac = min_vel_fac * 0.1
                            rospy.loginfo("Min speed reduced (now " 
                                          + str(userdata.actuator_speed * min_vel_fac)
                                          + "). [" + self.actuator_name + "]")
                    last_dist = dist
                    if (dist <= target_dist): # 0.04 deg
                        rospy.loginfo(">>>>>>>>>> Stopped at goal position.")
                        rospy.loginfo("goal position = " + str(self.actuator_goal_pos))
                        if userdata.encoder_goal:
                            rospy.loginfo("current position = " + str(self.current_encoder_pos))
                        else:
                            rospy.loginfo("current position = " + str(self.current_actuator_pos))
                        return 'success'
                    elif (dist <= max_dist) and (dist > min_dist):
                        m = (min_vel_fac - max_vel_fac) / (max_dist - min_dist)
                        step = userdata.actuator_speed * (m * (max_dist - dist) + max_vel_fac)
                        rospy.loginfo("Reducing velocity (vel = " + str(step) + ", dist = " + str(dist) + ") ... [" 
                                      + self.actuator_name + "]")
                    elif (dist <= min_dist):
                        if step > userdata.actuator_speed * min_vel_fac:
                            step = userdata.actuator_speed *min_vel_fac
                            rospy.loginfo("Moving with minimum velocity ... (" + str(dist) + ") [" 
                                          + self.actuator_name + "]")
                self.actuator_command.position = self.actuator_command.position + step * direction
                pub_actuator_command.publish(self.actuator_command)
                spin_rate.sleep()
        return 'error'