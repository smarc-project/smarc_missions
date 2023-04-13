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


def get_param(name, default=None):
    return rospy.get_param(rospy.search_param(name), default)

class LoloGotoWP(object):
    def __init__(self):

        self.lolo = Lolo(max_rpm = get_param("max_rpm", 2000),
                         max_fin_radians = get_param("max_fin_radians", 0.6))

        self.ros_lolo = ROSLolo(lolo = self.lolo,
                                robot_name = get_param("robot_name", "lolo"))

        self.update_freq = get_param("update_freq", 10)

        self.name = rospy.get_name()
        self.action_server = actionlib.SimpleActionServer(self.name,
                                                          GotoWaypointAction,
                                                          execute_cb = self.run,
                                                          auto_start=False)
        self.fb = GotoWaypointFeedback()
        self.result = GotoWaypointResult()
        self.goal = None

        self.tf_listener = tf.TransformListener()

    def start(self):
        rospy.loginfo("[{}] Started!".format(self.name))
        self.action_server.start()

    def on_preempt(self):
        self.goal = None
        self.fb.feedback_message = "[{}] Pre-empted!".format(self.name)
        return

    def update(self):
        self.lolo.control_yaw_from_desired_pos()

    def on_done(self):
        self.goal = None
        self.fb.feedback_message = "[{}] Completed!".format(self.name)
        pass

    def run(self, goal):
        # first, extract the position from the goal
        # and convert that to whatever reference frame the lolo model is in
        target_pos, _= self.tf_listener.transformPose(self.ros_lolo.reference_link, goal.pose)

        # acquire the depth
        if goal.z_control_mode == GotoWaypoint.Z_CONTROL_DEPTH:
            depth = goal.travel_depth
        # no other type of control yet
        else:
            rospy.logwarn("Lolo only does DEPTH control! Setting depth to 0")
            depth = 0

        self.lolo.set_desired_pos(target_pos.pose.x, target_pos.pose.y, depth)

        rate = rospy.Rate(self.update_freq)
        while True:
            if self.action_server.is_preempt_requested():
                self.on_preempt()
                self.action_server.set_preempted(self.result, "[{}] preempted!".format(self.name))
                return

            self.update()
            rate.sleep()

        # finished running
        self.on_done()
        self.action_server.set_succeeded(self.result, "[{}] Succeeded!")

if __name__ == "__main__":
    rospy.init_node("goto_waypoint")
    s = LoloGotoWP()
    s.start()
    rospy.loginfo("Spinning~")
    rospy.spin()
