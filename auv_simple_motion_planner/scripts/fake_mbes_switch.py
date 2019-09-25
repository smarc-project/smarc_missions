#!/usr/bin/python

# Copyright 2018 Nils Bore (nbore@kth.se)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
from std_msgs.msg import Float64, Header, Bool
import rospy
from sensor_msgs.msg import LaserScan

class FakeMBESSwitch(object):

    def mbes_control_cb(self, ctrl_msg):
        self.enable_mbes = ctrl_msg.data

    def mbes_cb(self, mbes_msg):
        if self.enable_mbes:
        	self.mbes_pub.publish(mbes_msg)

    def __init__(self):
    	self.gazebo_mbes_top = rospy.get_param('~mbes_gazebo', "mbes_laser")
    	self.enable_mbes_top = rospy.get_param('~enable_mbes', "enable_mbes")
    	self.output_mbes_top = rospy.get_param('~mbes_output', "mbes_data")

    	self.mbes_pub = rospy.Publisher(self.output_mbes_top, LaserScan, queue_size=10, latch=True)
	rospy.Subscriber(self.enable_mbes_top, Bool, self.mbes_control_cb)
	rospy.Subscriber(self.gazebo_mbes_top, LaserScan, self.mbes_cb)

	self.enable_mbes = False

	rospy.spin()

        

if __name__ == '__main__':
    rospy.init_node('mbes_control')
    planner = FakeMBESSwitch()
