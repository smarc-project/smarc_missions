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


from __future__ import division, print_function
import numpy as np
import rospy, tf

from smarc_msgs.msg import ThrusterFeedback, ThrusterRPM
from std_msgs.msg import Float32

class ROSLolo(object):
    def __init__(self,
                 lolo,
                 robot_name="lolo",
                 reference_link = "world_ned",
                 control_thrusters=True,
                 control_elevons=True,
                 control_rudder=True,
                 control_elevator=True):
        """
        Reads the state of a lolo from ROS topics and updates the given lolo object
        Reads the desired angles and such from the lolo object and publishes setpoints
        """
        self.lolo = lolo
        robot_name = "/"+robot_name
        self.base_link = robot_name + "/base_link"
        self.reference_link = reference_link

        self.tf_listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(self.reference_link, self.base_link, rospy.Time(0), rospy.Duration(5))
                break
            except:
                rospy.logwarn("Could not get tf between {} and {}, waiting until we do".format(self.reference_link, self.base_link))

        self.control_thrusters = control_thrusters
        if control_thrusters:
            self.t1_sub = rospy.Subscriber(robot_name+"/core/thruster1_fb", ThrusterFeedback, self.t1_cb, queue_size=1)
            self.t2_sub = rospy.Subscriber(robot_name+"/core/thruster2_fb", ThrusterFeedback, self.t2_cb, queue_size=1)
            self.t1_pub = rospy.Publisher(robot_name+"/core/thruster1_cmd", ThrusterRPM, queue_size=1)
            self.t2_pub = rospy.Publisher(robot_name+"/core/thruster2_cmd", ThrusterRPM, queue_size=1)

        self.control_elevons = control_elevons
        if control_elevons:
            self.elevon_p_sub = rospy.Subscriber(robot_name+"/core/elevon_port_fb", Float32, self.elevon_p_cb, queue_size=1)
            self.elevon_s_sub = rospy.Subscriber(robot_name+"/core/elevon_strb_fb", Float32, self.elevon_s_cb, queue_size=1)
            self.elevon_p_pub = rospy.Publisher(robot_name+"/core/elevon_port_cmd", Float32, queue_size=1)
            self.elevon_s_pub = rospy.Publisher(robot_name+"/core/elevon_strb_cmd", Float32, queue_size=1)

        self.control_rudder = control_rudder
        if control_rudder:
            self.rudder_sub = rospy.Subscriber(robot_name+"/core/rudder_fb", Float32, self.rudder_cb, queue_size=1)
            self.rudder_pub = rospy.Publisher(robot_name+"/core/rudder_cmd", Float32, queue_size=1)

        self.control_elevator = control_elevator
        if control_elevator:
            self.elevator_sub = rospy.Subscriber(robot_name+"/core/elevator_fb", Float32, self.elevator_cb, queue_size=1)
            self.elevator_pub = rospy.Publisher(robot_name+"/core/elevator_cmd", Float32, queue_size=1)



    def stop(self):
        rospy.loginfo("Stopping lolo controller")
        self.lolo.reset_goal()
        self.update()
        self.timer.shutdown()


    def start(self):
        rospy.loginfo("Starting lolo controller")
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)


    def update_tf(self):
        self.tf_listener.waitForTransform(self.reference_link, self.base_link, rospy.Time(0), rospy.Duration(1))
        trans, ori_quat = self.tf_listener.lookupTransform(self.reference_link, self.base_link, rospy.Time(0))
        self.lolo.update_pos(x = trans[0],
                             y = trans[1],
                             depth = trans[2])

        ori_rpy = tf.transformations.euler_from_quaternion(ori_quat)
        self.lolo.update_ori(r = ori_rpy[0],
                             p = ori_rpy[1],
                             y = ori_rpy[2])


    def update(self, timer_event=None):
        self.update_tf()

        if self.control_thrusters:
            self.t1_pub.publish(int(self.lolo.desired_rpms[0]))
            self.t2_pub.publish(int(self.lolo.desired_rpms[1]))

        if self.control_elevons:
            self.elevon_p_pub.publish(self.lolo.desired_elevon_angles[0])
            self.elevon_s_pub.publish(self.lolo.desired_elevon_angles[1])

        if self.control_rudder:
            self.rudder_pub.publish(self.lolo.desired_rudder_angle)

        if self.control_elevator:
            self.elevator_pub.publish(self.lolo.desired_elevator_angle)

    def t1_cb(self, msg):
        self.lolo.update_thruster_rpms(port=msg.rpm.rpm)

    def t2_cb(self, msg):
        self.lolo.update_thruster_rpms(strb=msg.rpm.rpm)

    def elevon_p_cb(self, msg):
        self.lolo.update_elevon_angles(port=msg.data)

    def elevon_s_cb(self, msg):
        self.lolo.update_elevon_angles(strb=msg.data)

    def rudder_cb(self, msg):
        self.lolo.update_rudder_angle(msg.data)

    def elevator_cb(self, msg):
        self.lolo.update_elevator_angle(msg.data)






