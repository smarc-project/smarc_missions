#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

# Copyright 2023 Ozer Ozkahraman (ozero@kth.se)
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
import geometry as geom
import rospy, tf, actionlib

from lolo import Lolo
from ros_lolo import ROSLolo


from smarc_msgs.msg import GotoWaypointFeedback, GotoWaypointResult, GotoWaypointAction, GotoWaypointGoal

class LoloGotoWP(object):
    def __init__(self,
                 lolo,
                 update_freq = 10,
                 name="LoloGotoWP"):
        self.lolo = lolo
        self.update_freq = update_freq
        self.name = name
        self.action_server = actionlib.SimpleActionServer(self.name,
                                                          GotoWaypointAction,
                                                          execute_cb = self.run,
                                                          auto_start=False)
        self.fb = GotoWaypointFeedback()
        self.result = GotoWaypointResult()
        self.goal = None

    def start(self):
        rospy.loginfo("[{}] Started!".format(self.name))
        self.action_server.start()

    def on_preempt(self):
        self.goal = None
        self.fb.feedback_message = "[{}] Pre-empted!".format(self.name)
        return

    def update(self):
        pass

    def on_done(self):
        self.goal = None
        self.fb.feedback_message = "[{}] Completed!".format(self.name)
        pass

    def run(self, goal):
        self.goal = goal
        while True:
            if self.action_server.is_preempt_requested():
                self.on_preempt()
                self.action_server.set_preempted(self.result, "[{}] preempted!".format(self.name))
                return

            self.update()

        # finished running
        self.on_done()
        self.action_server.set_succeeded(self.result, "[{}] Succeeded!")

if __name__ == "__main__":
    rospy.init_node("goto_waypoint")
    L = Lolo()
    RL = ROSLolo(L)

    L.set_desired_pos(40, 20)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        L.control_yaw_from_desired_pos()
        rate.sleep()

    RL.stop()
