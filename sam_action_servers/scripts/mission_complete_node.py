#! /usr/bin/env python

# Copyright 2021 Sriharsha Bhat (svbhat@kth.se)
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

#Action to disable controllers and return to surface once a mission is completed.

from __future__ import division, print_function

import actionlib
import rospy
import tf
from sam_msgs.msg import PercentStamped
from smarc_msgs.msg import ThrusterRPM
from std_msgs.msg import Float64, Header, Bool, Empty
#from move_base_msgs.msg import MoveBaseFeedback, MoveBaseResult, MoveBaseAction
from smarc_msgs.msg import GotoWaypointActionFeedback, GotoWaypointResult, GotoWaypointAction
from std_srvs.srv import SetBool
import time

from toggle_controller import ToggleController     

class MissionComplete(object):

    def mission_complete_cb(self,complete_msg):
        if not self.completed:    
            self.planned_surface()

    def planned_surface(self):
        
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

        self.completion_time = time.time()
        self.completed = True

        rospy.loginfo('%s: Mission completed, going to surface')

    def __init__(self, name):

        """Publish 0 to VBS and disable all controllers"""

        mission_complete_topic = rospy.get_param('~mission_complete_topic', '/sam/core/mission_complete')
        vbs_cmd_topic = rospy.get_param('~vbs_cmd_topic', '/sam/core/vbs_cmd')
        rpm_cmd_topic_1 = rospy.get_param('~rpm_cmd_topic_1', '/sam/core/thruster1_cmd')
        rpm_cmd_topic_2 = rospy.get_param('~rpm_cmd_topic_2', '/sam/core/thruster2_cmd')

        self.vbs_pub = rospy.Publisher(vbs_cmd_topic, PercentStamped, queue_size=1)
        self.rpm1_pub = rospy.Publisher(rpm_cmd_topic_1, ThrusterRPM, queue_size=1)
        self.rpm2_pub = rospy.Publisher(rpm_cmd_topic_2, ThrusterRPM, queue_size=1)

        self.mission_complete_sub = rospy.Subscriber(mission_complete_topic, Empty, self.mission_complete_cb )

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

        self.completed = False
        self.completion_time = 0


if __name__ == '__main__':

    rospy.init_node('mission_complete_node')
    surfacing_node = MissionComplete(rospy.get_name())

    r = rospy.Rate(11.) # 10hz

    while not rospy.is_shutdown():

        if surfacing_node.completed:
            time_diff = time.time()- surfacing_node.completion_time
            
            if time_diff > 30:
                surfacing_node.completed = False

        r.sleep()