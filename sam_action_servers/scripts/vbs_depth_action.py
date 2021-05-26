#! /usr/bin/env python

# Copyright 2018 Nils Bore, Sriharsha Bhat (nbore@kth.se, svbhat@kth.se)
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

from __future__ import division, print_function

import numpy as np
from smarc_msgs.msg import GotoWaypointActionFeedback, GotoWaypointResult, GotoWaypointAction, GotoWaypointGoal
import actionlib
import rospy
from std_msgs.msg import Float64
from toggle_controller import ToggleController     

     
class VBSDepth(object):

    # create messages that are used to publish feedback/result
    _feedback = GotoWaypointActionFeedback()
    _result = GotoWaypointResult()
    
    def depth_fb_cb(self,depth_feedback):
        self.depth_fb= depth_feedback.data

    def execute_cb(self, goal):

        rospy.loginfo("Goal received")

        r = rospy.Rate(11.) # 10hz
        counter = 0
        while not rospy.is_shutdown() and not self.at_depth:

            #self.toggle_yaw_ctrl.toggle(True)
            #self.toggle_depth_ctrl.toggle(True)
            
            # Preempted
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                # Stop VBS
                self.toggle_vbs_ctrl.toggle(False)

                print('VBS controller disengaged')
                self._as.set_preempted(self._result, "Preempted VBS action")
                return

            if abs(self.depth_fb-goal.travel_depth)<self.depth_tolerance:
                self.at_depth = True

            #Publishing to VBS
            rospy.loginfo_throttle(5,'Publishing to VBS')    
            depth_setpoint = goal.travel_depth
            self.toggle_vbs_ctrl.toggle(True)
            self.depth_pub.publish(depth_setpoint)

            r.sleep()
        
        if self.at_depth:
            self._result.reached_waypoint= True
            self._as.set_succeeded(self._result,"WP Reached")
            #self.toggle_vbs_ctrl.toggle(False)

        rospy.loginfo('%s: Succeeded' % self._action_name)


    def __init__(self, name):

        """Go to a waypoint with a POI, and perform a turbo turn around the POI"""
        self._action_name = name

        self.depth_tolerance = rospy.get_param('~depth_tolerance', 0.2)
        depth_setpoint_topic = rospy.get_param('~depth_setpoint_topic', '/sam/ctrl/vbs/setpoint')
        depth_feedback_topic = rospy.get_param('~depth_feedback_topic', '/sam/ctrl/depth_feedback')
        toggle_vbs_ctrl_service = rospy.get_param('~toggle_vbs_ctrl_service', '/sam/ctrl/toggle_vbs_ctrl')
        self.toggle_vbs_ctrl = ToggleController(toggle_vbs_ctrl_service, False)

        rospy.Subscriber(depth_feedback_topic, Float64, self.depth_fb_cb)
        self.depth_pub = rospy.Publisher(depth_setpoint_topic, Float64, queue_size=10)
        self.at_depth = False

        self._as = actionlib.SimpleActionServer(self._action_name, GotoWaypointAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Announced action server with name: %s", self._action_name)

        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('vbs_depth')
    planner = VBSDepth(rospy.get_name())
