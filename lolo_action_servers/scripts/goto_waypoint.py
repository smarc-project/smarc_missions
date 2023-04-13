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


from smarc_msgs.msg import GotoWaypoint
from smarc_msgs.msg import GotoWaypointFeedback, GotoWaypointResult, GotoWaypointAction


def get_param(name, default=None):
    return rospy.get_param(rospy.search_param(name), default)

class LoloGotoWP(object):
    def __init__(self):

        self.lolo = Lolo(max_rpm = get_param("max_rpm", 2000),
                         max_fin_radians = get_param("max_fin_radians", 0.6))

        self.ros_lolo = ROSLolo(lolo = self.lolo,
                                robot_name = get_param("robot_name", "lolo"))

        self.update_freq = get_param("update_freq", 10)

        self.tf_listener = tf.TransformListener()

        self.name = rospy.get_name()
        self.reset_fb_result()
        self.action_server = actionlib.SimpleActionServer(self.name,
                                                          GotoWaypointAction,
                                                          execute_cb = self.run,
                                                          auto_start = False)

    ###################################################
    # do something every tick here
    # ideally you shouldnt need to think about ros at all
    # inside this funtion
    ###################################################
    def update(self):
        self.lolo.control_yaw_from_goal()

    ###################################################
    # action server piping, shouldnt need modification most of the time
    ###################################################
    def feedback(self, msg):
        s = "{} [{}]".format(msg, self.name)
        self.fb.feedback_message = s
        rospy.loginfo_throttle(5, s)
        self.action_server.publish_feedback(self.fb)

    def start(self):
        self.action_server.start()
        rospy.loginfo("Started!")

    def on_preempt(self):
        self.ros_lolo.stop()
        self.feedback("Pre-empted!")
        self.action_server.set_preempted(self.result, "[{}] preempted!".format(self.name))
        self.reset_fb_result()

    def on_done(self):
        self.ros_lolo.stop()
        self.feedback("Completed!")
        self.result.reached_waypoint = True
        self.action_server.set_succeeded(self.result, "[{}] Succeeded!")
        self.reset_fb_result()

    def on_new_goal(self):
        self.feedback("Got goal!")
        self.reset_fb_result()
        self.ros_lolo.start()

    def reset_fb_result(self):
        self.fb = GotoWaypointFeedback()
        self.result = GotoWaypointResult()

    def run(self, goal):
        self.on_new_goal()
        # first, extract the position from the goal
        # and convert that to whatever reference frame the lolo model is in
        wp = goal.waypoint
        goal_pose_stamped = self.tf_listener.transformPose(target_frame = self.ros_lolo.reference_link,
                                                           ps = wp.pose)
        target_posi = goal_pose_stamped.pose.position

        # acquire the depth
        if wp.z_control_mode == GotoWaypoint.Z_CONTROL_DEPTH:
            depth = wp.travel_depth
        # no other type of control yet
        else:
            rospy.logwarn("Lolo only does DEPTH control! Setting depth to 0")
            depth = 0

        # acquire the rpm
        if wp.speed_control_mode == GotoWaypoint.SPEED_CONTROL_RPM:
            rpm = wp.travel_rpm
        else:
            rospy.logwarn("Lolo only does RPM control! Setting rpm to 0")
            rpm = 0

        tolerance = wp.goal_tolerance
        if tolerance < 0.5:
            rospy.logwarn("Goal tolerance is too small, lolo is not a surgeon! Setting to 0.5m")
            tolerance = 0.5

        # set internal goal from message params
        self.lolo.set_goal(x = target_posi.x,
                           y = target_posi.y,
                           depth = depth,
                           rpm = rpm)

        # and finally, we start spinning and controlling things
        rate = rospy.Rate(self.update_freq)
        while not rospy.is_shutdown():
            if self.action_server.is_preempt_requested():
                self.on_preempt()
                # return, not break!
                return

            xy_dist = self.lolo.xy_diff_to_goal
            self.feedback("trgt:{}, XYdist:{:.1f}, tol:{:.1f}".format(wp.name, xy_dist, tolerance))
            if  xy_dist <= tolerance:
                # success~
                break

            self.update()
            rate.sleep()

        # finished running
        self.on_done()

if __name__ == "__main__":
    rospy.init_node("goto_waypoint")
    s = LoloGotoWP()
    s.start()
    rospy.loginfo("Spinning~")
    rospy.spin()
