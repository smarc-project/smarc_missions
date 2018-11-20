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

from __future__ import division, print_function

import scipy.special
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64, Header, Bool
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
import rospy
import tf

class BezierController(object):

    def callback(self, path_msg):

        self.current_path = path_msg.poses
        self.path_idx = 0

    def control(self):

        if self.current_path is None or len(self.current_path) == 0:
            self.current_path = None
            return 0., 0.

        try:
            (trans, rot) = self.listener.lookupTransform("/world",  self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 0., 0.

        p = np.array(trans)
        euler = tf.transformations.euler_from_quaternion(rot)
        pitch = euler[1]
        yaw = euler[2]
        heading = np.array([np.cos(yaw)*np.cos(pitch), np.sin(yaw)*np.cos(pitch), -np.sin(pitch)])
        lq = np.array([self.current_path[-1].pose.position.x,
                       self.current_path[-1].pose.position.y,
                       self.current_path[-1].pose.position.z])
        if (lq - p).dot(heading) < 0.:
            self.current_path = None
            return 0., 0.
    
        j = min(self.path_idx, len(self.current_path)-1)
        while True:
            q = np.array([self.current_path[j].pose.position.x,
                          self.current_path[j].pose.position.y,
                          self.current_path[j].pose.position.z])
            if np.linalg.norm(p - q) > self.carrot_dist or j >= len(self.current_path)-1:
                break
            j = j + 1
        self.path_idx = j
        
        # This is probably the way to go for stopping when we have a better planner
        #if (q - p).dot(heading) < 0.:
        #    self.current_path = None
        #    return 0., 0.

        dist = np.linalg.norm(p - q)
        fp = p + dist*heading

        target_pose = PoseStamped()
        target_pose.header.frame_id = "/world"
        target_pose.pose.position.x = q[0]
        target_pose.pose.position.y = q[1]
        target_pose.pose.position.z = q[2]
        self.target_pub.publish(target_pose)

        course_pose = PoseStamped()
        course_pose.header.frame_id = "/world"
        course_pose.pose.position.x = fp[0]
        course_pose.pose.position.y = fp[1]
        course_pose.pose.position.z = fp[2]
        self.course_pub.publish(course_pose)

        z_offset = q[2] - fp[2]
        q[2] = 0.
        p[2] = 0.
        v = fp - p;
        v = 1./np.linalg.norm(v)*v
        nv = np.array([-v[1], v[0], 0.])

        lateral_offset = np.dot(nv, q-p)

        return lateral_offset, z_offset

    def vert_control_cb(self, value):

	header = Header()
	
        value = value.data
        self.vert_fin0.publish(header, value)
	self.vert_fin1.publish(header, value)
	self.vert_fin2.publish(header, -value)
	self.vert_fin3.publish(header, -value)

    def hor_control_cb(self, value):

	header = Header()
	    
        value = value.data
        self.hor_fin0.publish(header, -value)
	self.hor_fin1.publish(header, value)
	self.back_fin.publish(header, value)

    def __init__(self):
        
        """Plot an example bezier curve."""
        
        self.carrot_dist = rospy.get_param('~carrot_dist', 10.)
        self.base_frame = rospy.get_param('~base_frame', "lolo_auv_1/base_link")
        self.thrust_level = rospy.get_param('~thrust_level', 200.)
        
        self.current_path = None
        self.path_idx = 0.

        self.listener = tf.TransformListener()

        setpoint_pub = rospy.Publisher('/setpoint', Float64, queue_size=10, latch=True)
        lateral_offset_pub = rospy.Publisher('/vertical_fins/state', Float64, queue_size=10)
        z_offset_pub = rospy.Publisher('/horizontal_fins/state', Float64, queue_size=10)
        self.target_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
        self.course_pub = rospy.Publisher('/course_pose', PoseStamped, queue_size=10)

        self.auv_name = "/lolo_auv_1"

	thruster0 = rospy.Publisher(self.auv_name + '/thrusters/0/input', FloatStamped, queue_size=10)
	thruster1 = rospy.Publisher(self.auv_name + '/thrusters/1/input', FloatStamped, queue_size=10)

        self.vert_fin0 = rospy.Publisher(self.auv_name + '/fins/1/input', FloatStamped, queue_size=10)
	self.vert_fin1 = rospy.Publisher(self.auv_name + '/fins/0/input', FloatStamped, queue_size=10)
	self.vert_fin2 = rospy.Publisher(self.auv_name + '/fins/2/input', FloatStamped, queue_size=10)
	self.vert_fin3 = rospy.Publisher(self.auv_name + '/fins/3/input', FloatStamped, queue_size=10)

	self.hor_fin0 = rospy.Publisher(self.auv_name + '/fins/4/input', FloatStamped, queue_size=10)
	self.hor_fin1 = rospy.Publisher(self.auv_name + '/fins/5/input', FloatStamped, queue_size=10)
	self.back_fin = rospy.Publisher(self.auv_name + '/back_fins/0/input', FloatStamped, queue_size=10)

        vertical_pid_enable = rospy.Publisher('/vertical_fins/pid_enable', Bool, queue_size=10)
        horizontal_pid_enable = rospy.Publisher('/horizontal_fins/pid_enable', Bool, queue_size=10)

        rospy.Subscriber('/vertical_fins/control_effort', Float64, self.vert_control_cb)
        rospy.Subscriber('/horizontal_fins/control_effort', Float64, self.hor_control_cb)
        rospy.Subscriber('/global_plan', Path, self.callback)

        vertical_pid_enable.publish(False)
        horizontal_pid_enable.publish(False)
        setpoint_pub.publish(0.)
        
        header = Header()
        r = rospy.Rate(10) # 10hz
        did_reset = False
        while not rospy.is_shutdown():
            lateral_offset, z_offset = self.control()
            if self.current_path is None and not did_reset:
                thruster0.publish(header, 0.)
                thruster1.publish(header, 0.)
                vertical_pid_enable.publish(False)
                horizontal_pid_enable.publish(False)
                did_reset = True
            elif self.current_path is not None:
                vertical_pid_enable.publish(True)
                horizontal_pid_enable.publish(True)
                lateral_offset_pub.publish(lateral_offset)
                z_offset_pub.publish(z_offset)
                thruster0.publish(header, self.thrust_level)
                thruster1.publish(header, self.thrust_level)
                did_reset = False
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('bezier_controller')
    planner = BezierController()
