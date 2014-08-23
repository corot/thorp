#!/usr/bin/env python

"""
  ir_ranger.py - convert analog stream into range measurements
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy

from sensor_msgs.msg import Range
from arbotix_msgs.msg import Analog

from sensors import *

class ir_ranger:
    def __init__(self):
        rospy.init_node("ir_ranger")
        
        # sensor type: choices are 2Y0A21 (10-90 cm), more to come if needed
        self.sensor_t = rospy.get_param("~type", "2Y0A21")
        if self.sensor_t == "2Y0A21":
            self.sensor = GP2Y0A21()
        else:
            rospy.logerr('Sensor type',self.sensor_t,'not implemented')
            return 

        self.topic_name = rospy.get_param("~name")

        # setup a range message to use
        self.msg = Range()
        self.msg.radiation_type = self.sensor.radiation_type
        self.msg.field_of_view = self.sensor.field_of_view
        self.msg.min_range = self.sensor.min_range
        self.msg.max_range = self.sensor.max_range
        
        # publish/subscribe
        self.pub = rospy.Publisher("ir_range", Range, queue_size=5)
        rospy.Subscriber("arbotix/" + self.topic_name, Analog, self.readingCb)

        rospy.spin()
        
    def readingCb(self, msg):
        # convert msg.value into range.range
        self.msg.header.stamp = rospy.Time.now()
        self.msg.range = self.sensor.convert(msg.value)
        self.pub.publish(self.msg)

if __name__=="__main__":
    ir_ranger()
