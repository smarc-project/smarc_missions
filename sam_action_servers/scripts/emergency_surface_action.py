#! /usr/bin/env python

# Copyright 2020 Sriharsha Bhat (svbhat@kth.se)
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

#Subscribe to the emergency surface topic. Send commands to the abort flag if there is an issue.

from __future__ import division, print_function

import actionlib
import rospy
from sam_msgs.msg import PercentStamped
from smarc_msgs.msg import ThrusterRPM
from std_msgs.msg import Bool
#from move_base_msgs.msg import MoveBaseFeedback, MoveBaseResult, MoveBaseAction
from smarc_bt.msg import GotoWaypointActionFeedback, GotoWaypointResult, GotoWaypointAction
from toggle_controller import ToggleController     

class EmergencySurface(object):

    def execute_cb(self, goal):

        rospy.loginfo("Emergency action initiated")

        r = rospy.Rate(11.) # 10hz
        while not rospy.is_shutdown():

            # Preempted
            if self._as.is_preempt_requested():
                # Publish emergency command
                self.emergency_pub.publish(False)

                #Enable controllers
                '''self.toggle_pitch_ctrl.toggle(True)
                self.toggle_vbs_ctrl.toggle(True)
                self.toggle_tcg_ctrl.toggle(True)
                self.toggle_yaw_ctrl.toggle(True)
                self.toggle_depth_ctrl.toggle(True)
                self.toggle_speed_ctrl.toggle(True)
                self.toggle_roll_ctrl.toggle(True)'''
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted(GotoWaypointResult(), "Preempted EmergencySurface action")
                return

            # Publish emergency command
            self.emergency_pub.publish(True)

            #Disable controllers
            self.toggle_pitch_ctrl.toggle(False)
            self.toggle_vbs_ctrl.toggle(False)
            self.toggle_tcg_ctrl.toggle(False)
            self.toggle_yaw_ctrl.toggle(False)
            self.toggle_depth_ctrl.toggle(False)
            self.toggle_speed_ctrl.toggle(False)
            self.toggle_roll_ctrl.toggle(False)

            #set VBS to 0
            vbs_level = PercentStamped()
            vbs_level.value = 0.0
            self.vbs_pub.publish(vbs_level)

            # Stop thrusters
            rpm1 = ThrusterRPM()
            rpm2 = ThrusterRPM()
            rpm1.rpm = 0
            rpm2.rpm = 0
            self.rpm1_pub.publish(rpm1)
            self.rpm2_pub.publish(rpm2)

            r.sleep()

        rospy.loginfo('%s: Completed' % self._action_name)

    #def timer_callback(self, event):

    def __init__(self, name):

        """Publish 0 to VBS and disable all controllers"""
        self._action_name = name

        emergency_topic = rospy.get_param('~emergency_topic', '/sam/abort')
        vbs_cmd_topic = rospy.get_param('~vbs_cmd_topic', '/sam/core/vbs_cmd')
        rpm_cmd_topic_1 = rospy.get_param('~rpm_cmd_topic_1', '/sam/core/thruster1_cmd')
        rpm_cmd_topic_2 = rospy.get_param('~rpm_cmd_topic_2', '/sam/core/thruster2_cmd')


        #rospy.Timer(rospy.Duration(2), self.timer_callback)
        self.emergency_pub = rospy.Publisher(emergency_topic, Bool, queue_size=10)
        self.vbs_pub = rospy.Publisher(vbs_cmd_topic, PercentStamped, queue_size=10)
        self.rpm1_pub = rospy.Publisher(rpm_cmd_topic_1, ThrusterRPM, queue_size=10)
        self.rpm2_pub = rospy.Publisher(rpm_cmd_topic_2, ThrusterRPM, queue_size=10)

        #controller services
        toggle_yaw_ctrl_service = rospy.get_param('~toggle_yaw_ctrl_service', '/sam/ctrl/toggle_yaw_ctrl')
        toggle_depth_ctrl_service = rospy.get_param('~toggle_depth_ctrl_service', '/sam/ctrl/toggle_depth_ctrl')
        toggle_vbs_ctrl_service = rospy.get_param('~toggle_vbs_ctrl_service', '/sam/ctrl/toggle_vbs_ctrl')
        toggle_speed_ctrl_service = rospy.get_param('~toggle_speed_ctrl_service', '/sam/ctrl/toggle_speed_ctrl')
        toggle_roll_ctrl_service = rospy.get_param('~toggle_roll_ctrl_service', '/sam/ctrl/toggle_roll_ctrl')
        toggle_pitch_ctrl_service = rospy.get_param('~toggle_pitch_ctrl_service', '/sam/ctrl/toggle_pitch_ctrl')
        toggle_tcg_ctrl_service = rospy.get_param('~toggle_tcg_ctrl_service', '/sam/ctrl/toggle_tcg_ctrl')

        self.toggle_yaw_ctrl = ToggleController(toggle_yaw_ctrl_service, False)
        self.toggle_depth_ctrl = ToggleController(toggle_depth_ctrl_service, False)
        self.toggle_vbs_ctrl = ToggleController(toggle_vbs_ctrl_service, False)
        self.toggle_speed_ctrl = ToggleController(toggle_speed_ctrl_service, False)
        self.toggle_roll_ctrl = ToggleController(toggle_roll_ctrl_service, False)
        self.toggle_pitch_ctrl = ToggleController(toggle_pitch_ctrl_service, False)
        self.toggle_tcg_ctrl = ToggleController(toggle_tcg_ctrl_service, False)

        self._as = actionlib.SimpleActionServer(self._action_name, GotoWaypointAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        rospy.loginfo("Announced action server with name: %s", self._action_name)

        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('emergency_surface_action')
    planner = EmergencySurface(rospy.get_name())
