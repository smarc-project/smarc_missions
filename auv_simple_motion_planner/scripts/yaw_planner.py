#!/usr/bin/python

# Copyright original bezier planner code (c) 2016 Atsushi Sakai,

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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
from sam_msgs.msg import ThrusterRPMs

class YawPlanner(object):

    # create messages that are used to publish feedback/result
    _feedback = MoveBaseFeedback()
    _result = MoveBaseResult()
    
    def execute_cb(self, goal):
        # helper variables
        #r = rospy.Rate(1)
        rospy.loginfo("Goal received")
        
        success = True
        self.nav_goal = goal.target_pose.pose
        
        r = rospy.Rate(10.) # 10hz
        counter = 0
        while not rospy.is_shutdown() and self.nav_goal is not None:
            # Preempted
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                self.nav_goal = None

                # Stop thrusters
                rpm = ThrusterRPMs()
                rpm.thruster_1_rpm = 0.
                self.rpm_pub.publish(rpm)
                break

            # Publish feedback
            if counter % 100 == 0:
                try:
                    (trans, rot) = self.listener.lookupTransform("/world", "sam/base_link", rospy.Time(0))
                    pose_fb = PoseStamped()
                    pose_fb.header.frame_id = "/world"
                    pose_fb.pose.position.x = trans[0]
                    pose_fb.pose.position.y = trans[1]
                    pose_fb.pose.position.z = trans[2]
                    self._feedback.base_position = pose_fb
                    self._feedback.base_position.header.stamp = rospy.get_rostime()
                    self._as.publish_feedback(self._feedback)

                    rospy.loginfo("Sending feedback")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("Error with tf")
                    continue

            # Thruster forward
            rpm = ThrusterRPMs()
            rpm.thruster_1_rpm = 1000.
            self.rpm_pub.publish(rpm)
            rospy.loginfo("Thrusters forward")

            counter += 1
            r.sleep()
        
        if success:
            # Stop thruster
            rpm = ThrusterRPMs()
            rpm.thruster_1_rpm = -1000.0
            
            cnt = 0
            while not rospy.is_shutdown() and cnt < 50:
                self.rpm_pub.publish(rpm)
                cnt += 1
                r.sleep()


            #self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


    def timer_callback(self, event):
        if self.nav_goal is None:
            rospy.loginfo("Nav goal is None!")
            return
        
        try:
            (trans, rot) = self.listener.lookupTransform("/world", self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # TODO: we could use this code for the other check also
        goal_point = PointStamped()
        goal_point.header.frame_id = "/world"
        goal_point.header.stamp = rospy.Time(0)
        goal_point.point.x = self.nav_goal.position.x
        goal_point.point.y = self.nav_goal.position.y
        goal_point.point.z = self.nav_goal.position.z
        try:
            goal_point_base = self.listener.transformPoint(self.base_frame, goal_point)
            if goal_point_base.point.x < 0.:
                rospy.loginfo("Ahead of goal, returning success!")
                self.nav_goal = None
                return
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        #print("Checking if nav goal is reached!")

        current_pos = np.array(trans)
        current_pos[2] = 0.0
        end_pos = np.array([self.nav_goal.position.x, self.nav_goal.position.y, 0.])
        if np.linalg.norm(current_pos - end_pos) < self.goal_tolerance:
            rospy.loginfo("Reached goal!")
            self.nav_goal = None
        #else:
        #    print("Did not reach nav goal!")

    def __init__(self, name):
        
        """Plot an example bezier curve."""
        self._action_name = name
        
        self.heading_offset = rospy.get_param('~heading_offsets', 5.)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 5.)
        self.base_frame = rospy.get_param('~base_frame', "lolo_auv_1/base_link")

        self.nav_goal = None

        self.listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(2), self.timer_callback)

        self.rpm_pub = rospy.Publisher('/uavcan_rpm_command', ThrusterRPMs, queue_size=10)
        self._as = actionlib.SimpleActionServer(self._action_name, MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Announced action server with name: %s", self._action_name)
        
        r = rospy.Rate(10) # 10hz

        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('p2p_planner')
    planner = P2PPlanner(rospy.get_name())
