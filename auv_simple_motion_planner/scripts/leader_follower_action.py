#!/usr/bin/python

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
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseResult, MoveBaseAction
import actionlib
import rospy
import tf
from sam_msgs.msg import ThrusterRPMs, ThrusterAngles
from std_msgs.msg import Float64, Header, Bool
import math

class LeaderFollower(object):

    # create messages that are used to publish feedback/result
    _feedback = MoveBaseFeedback()
    _result = MoveBaseResult()

    def execute_cb(self, goal):

        rospy.loginfo_throttle(5, "Goal received")

        success = True
        rate = rospy.Rate(11.) # 10hz
        counter = 0
        while not rospy.is_shutdown():

            self.yaw_pid_enable.publish(True)
            self.depth_pid_enable.publish(True)
            # Preempted
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                success = False

                # Stop thrusters
                self.rpm.thruster_1_rpm = 0.
                self.rpm.thruster_2_rpm = 0.
                self.rpm_pub.publish(self.rpm)
                self.yaw_pid_enable.publish(False)
                self.depth_pid_enable.publish(False)
                self.vel_pid_enable.publish(False)

                print('leader_follower_action: stopped thrusters')
                self._as.set_preempted(self._result, "Preempted WP action")
                return

            # Compute and Publish setpoints
            if counter % 1 == 0:
                # distance check is done in the BT, we will add CBFs here later, which will include
                # that distance as a constraint anyways
                #  if sqrt(rel_trans[0]**2 + rel_trans[1]**2 + rel_trans[2]**2) < self.min_dist:
                    #  break
                #Check transform between SAM1 (leader- goal) and SAM2 (follower -self), define a goal point and call the go_to_point() function
                self.leader_frame = goal.target_pose.header.frame_id

                try:
                    (follower_trans, follower_rot) = self.listener.lookupTransform(self.follower_odom,
                                                                                   self.follower_frame,
                                                                                   rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException):
                    rospy.logwarn_throttle_identical(5, "Could not get transform between "+ self.leader_frame +" and "+ self.follower_frame)
                    success = False
                    break
                except:
                    rospy.logwarn_throttle_identical(5, "Could not do tf lookup for some other reason")
                    success = False
                    break

                try:
                    (leader_trans, leader_rot) = self.listener.lookupTransform(self.follower_odom,
                                                                               self.leader_frame,
                                                                               rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException):
                    rospy.logwarn_throttle_identical(5, "Could not get transform between "+ self.leader_frame +" and "+ self.follower_frame)
                    success = False
                    break
                except:
                    rospy.logwarn_throttle_identical(5, "Could not do tf lookup for some other reason")
                    success = False
                    break
                
                xdiff = leader_trans[0]-follower_trans[0]
                ydiff = leader_trans[1]-follower_trans[1]
                zdiff = leader_trans[2]-follower_trans[2]
                yaw_setpoint = math.atan2(ydiff,xdiff)
                depth_setpoint = -zdiff 
                rospy.loginfo_throttle(10,'yaw_setpoint:',yaw_setpoint, 'depth_setpoint:',depth_setpoint)

                self.depth_pub.publish(depth_setpoint)

                if self.vel_ctrl_flag:
                    rospy.loginfo_throttle_identical(5, "vel ctrl, no turbo turn")
                    #with Velocity control
                    self.yaw_pid_enable.publish(True)
                    self.yaw_pub.publish(yaw_setpoint)
                    # Publish to velocity controller
                    self.vel_pid_enable.publish(True)
                    self.vel_pub.publish(self.vel_setpoint)
                    self.roll_pub.publish(self.roll_setpoint)
                    #rospy.loginfo("Velocity published")
                else:
                    #turbo turn not included, no velocity control
                    rospy.loginfo_throttle_identical(5, "Normal WP following, no turbo turn")
                    self.yaw_pid_enable.publish(True)
                    self.yaw_pub.publish(yaw_setpoint)
                    # Thruster forward
                    self.rpm.thruster_1_rpm = self.forward_rpm
                    self.rpm.thruster_2_rpm = self.forward_rpm
                    self.rpm_pub.publish(self.rpm)
                    #rospy.loginfo("Thrusters forward")

            counter += 1
            rate.sleep()

        # Stop thruster
        self.vel_pid_enable.publish(False)
        self.rpm.thruster_1_rpm = 0.0
        self.rpm.thruster_2_rpm = 0.0
        self.rpm_pub.publish(self.rpm)

        #Stop controllers
        self.yaw_pid_enable.publish(False)
        self.depth_pid_enable.publish(False)
        self.vel_pid_enable.publish(False)
        if self._result:
            rospy.loginfo('%s: Succeeded' % self._action_name)
        else:
            rospy.logwarn_throttle_identical(3, '%s: Failed' % self._action_name)
        self._as.set_succeeded(self._result)


    def __init__(self, name):

        self.rpm = ThrusterRPMs()

        """Publish yaw and depth setpoints based on waypoints"""
        self._action_name = name

        self.follower_frame = rospy.get_param('~follower_frame', '/sam_2/base_link')
        self.follower_odom = rospy.get_param('~follower_odom', '/sam_2/odom')
        #  self.min_dist = rospy.get_param('~min_dist', 5.)

        rpm_cmd_topic = rospy.get_param('~rpm_cmd_topic', '/sam/core/rpm_cmd')
        heading_setpoint_topic = rospy.get_param('~heading_setpoint_topic', '/sam/ctrl/dynamic_heading/setpoint')
        yaw_pid_enable_topic = rospy.get_param('~yaw_pid_enable_topic', '/sam/ctrl/dynamic_heading/pid_enable')
        depth_setpoint_topic = rospy.get_param('~depth_setpoint_topic', '/sam/ctrl/dynamic_depth/setpoint')
        depth_pid_enable_topic = rospy.get_param('~depth_pid_enable_topic', '/sam/ctrl/dynamic_depth/pid_enable')

        self.forward_rpm = int(rospy.get_param('~forward_rpm', 1000))

        #related to velocity regulation instead of rpm
        self.vel_ctrl_flag = rospy.get_param('~vel_ctrl_flag', False)
        self.vel_setpoint = rospy.get_param('~vel_setpoint', 0.5) #velocity setpoint in m/s
        self.roll_setpoint = rospy.get_param('~roll_setpoint', 0)
        vel_setpoint_topic = rospy.get_param('~vel_setpoint_topic', '/sam/ctrl/dynamic_velocity/u_setpoint')
        roll_setpoint_topic = rospy.get_param('~roll_setpoint_topic', '/sam/ctrl/dynamic_velocity/roll_setpoint')
        vel_pid_enable_topic = rospy.get_param('~vel_pid_enable_topic', '/sam/ctrl/dynamic_velocity/pid_enable')
        self.listener = tf.TransformListener()

        self.rpm_pub= rospy.Publisher(rpm_cmd_topic, ThrusterRPMs, queue_size=10)
        self.yaw_pub = rospy.Publisher(heading_setpoint_topic, Float64, queue_size=10)
        self.depth_pub = rospy.Publisher(depth_setpoint_topic, Float64, queue_size=10)
        self.vel_pub = rospy.Publisher(vel_setpoint_topic, Float64, queue_size=10)
        self.roll_pub = rospy.Publisher(roll_setpoint_topic, Float64, queue_size=10)
        self.yaw_pid_enable = rospy.Publisher(yaw_pid_enable_topic, Bool, queue_size=10)
        self.depth_pid_enable = rospy.Publisher(depth_pid_enable_topic, Bool, queue_size=10)
        self.vel_pid_enable = rospy.Publisher(vel_pid_enable_topic, Bool, queue_size=10)

        self._as = actionlib.SimpleActionServer(self._action_name, MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Announced action server with name: %s", self._action_name)

        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('leader_follower_action')
    planner = LeaderFollower(rospy.get_name())
