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

import scipy.special
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseResult, MoveBaseAction
import actionlib
import rospy
import tf

def calc_4points_bezier_path(svec, syaw, spitch, evec, eyaw, epitch, offset, n_points=100):
    """
    Compute control points and path given start and end position.

    :param sx: (float) x-coordinate of the starting point
    :param sy: (float) y-coordinate of the starting point
    :param syaw: (float) yaw angle at start
    :param ex: (float) x-coordinate of the ending point
    :param ey: (float) y-coordinate of the ending point
    :param eyaw: (float) yaw angle at the end
    :param offset: (float)
    :return: (numpy array, numpy array)
    """
    #dist = np.linalg.norm(svec - evec) / offset
    dist = offset
    control_points = np.array(
        (svec,
         svec + dist*np.array([np.cos(syaw)*np.cos(spitch), np.sin(syaw)*np.cos(spitch), np.sin(spitch)]),
         evec - dist*np.array([np.cos(eyaw)*np.cos(epitch), np.sin(eyaw)*np.cos(epitch), np.sin(epitch)]),
         evec))

    path = calc_bezier_path(control_points, n_points=100)

    return path, control_points


def calc_bezier_path(control_points, n_points=100):
    """
    Compute bezier path (trajectory) given control points.

    :param control_points: (numpy array)
    :param n_points: (int) number of points in the trajectory
    :return: (numpy array)
    """
    traj = []
    for t in np.linspace(0, 1, n_points):
        traj.append(bezier(t, control_points))

    return np.array(traj)


def bernstein_poly(n, i, t):
    """
    Bernstein polynom.

    :param n: (int) polynom degree
    :param i: (int)
    :param t: (float)
    :return: (float)
    """
    return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)
    #return scipy.misc.comb(n, i) * t ** i * (1 - t) ** (n - i)


def bezier(t, control_points):
    """
    Return one point on the bezier curve.

    :param t: (float) number in [0, 1]
    :param control_points: (numpy array)
    :return: (numpy array) Coordinates of the point
    """
    n = len(control_points) - 1
    return np.sum([bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)


def bezier_derivatives_control_points(control_points, n_derivatives):
    """
    Compute control points of the successive derivatives of a given bezier curve.

    A derivative of a bezier curve is a bezier curve.
    See https://pomax.github.io/bezierinfo/#derivatives
    for detailed explanations

    :param control_points: (numpy array)
    :param n_derivatives: (int)
    e.g., n_derivatives=2 -> compute control points for first and second derivatives
    :return: ([numpy array])
    """
    w = {0: control_points}
    for i in range(n_derivatives):
        n = len(w[i])
        w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j]) for j in range(n - 1)])
    return w

class BezierPlanner(object):

    # create messages that are used to publish feedback/result
    _feedback = MoveBaseFeedback()
    _result = MoveBaseResult()

    # def callback(self, pose_msg):

    #     if len(pose_msg.header.frame_id) == 0:
    #         self.nav_goal = None
    #         return

    #     self.nav_goal = pose_msg.pose
    #     path, pose = self.plan()
    #     self.pub.publish(path)
    
    def execute_cb(self, goal):

        success = True
        self.nav_goal = goal.target_pose.pose

        r = rospy.Rate(10.) # 10hz
        counter = 0
        while not rospy.is_shutdown() and self.nav_goal is not None:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                self.nav_goal = None
                break
            if counter % 100 == 0:
                path, pose = self.plan()
                self.pub.publish(path)
                self._feedback.base_position = pose
                self._feedback.base_position.header.stamp = rospy.get_rostime()
                self._as.publish_feedback(self._feedback)
            counter += 1
            r.sleep()
        
        self.pub.publish(Path())

        if success:
            #self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def plan(self):

        try:
            (trans, rot) = self.listener.lookupTransform("/world", self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return Path(), PoseStamped()

        start_pos = np.array(trans)
        euler = tf.transformations.euler_from_quaternion(rot)
        start_pitch = euler[1] #np.radians(-40.0)  # [rad]
        start_yaw = euler[2] # np.radians(180.0)  # [rad]

        end_pos = np.array([self.nav_goal.position.x, self.nav_goal.position.y, self.nav_goal.position.z])

        #if np.linalg.norm(start_pos - end_pos) < self.goal_tolerance:
        #    rospy.loginfo("Reached goal!")
        #    self.nav_goal = None
        #    return Path()

        end_rot = [self.nav_goal.orientation.x, self.nav_goal.orientation.y, self.nav_goal.orientation.z, self.nav_goal.orientation.w]
        euler = tf.transformations.euler_from_quaternion(end_rot)
        end_pitch = euler[1]
        end_yaw = euler[2]

        curve, control_points = calc_4points_bezier_path(
            start_pos, start_yaw, start_pitch, end_pos, end_yaw, end_pitch, self.heading_offset, n_points=self.n_points)

        #derivatives_cp = bezier_derivatives_control_points(control_points, 1)

        path = Path()
        path.header.frame_id = "/world"
        path.header.stamp = rospy.get_rostime()
        for i in range(0, self.n_points):
            pose = PoseStamped()
            pose.pose.position.x = curve.T[0][i]
            pose.pose.position.y = curve.T[1][i]
            pose.pose.position.z = curve.T[2][i]
            path.poses.append(pose)

        pose = PoseStamped()
        pose.pose.position.x = start_pos[0]
        pose.pose.position.y = start_pos[1]
        pose.pose.position.z = start_pos[2]

        return path, pose

    def timer_callback(self, event):

        if self.nav_goal is None:
            #print("Nav goal is None!")
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

        start_pos = np.array(trans)
        end_pos = np.array([self.nav_goal.position.x, self.nav_goal.position.y, -85.])
        if np.linalg.norm(start_pos - end_pos) < self.goal_tolerance:
            rospy.loginfo("Reached goal!")
            self.nav_goal = None
        #else:
        #    print("Did not reach nav goal!")

    def __init__(self, name):
        
        """Plot an example bezier curve."""
        self._action_name = name
        
        self.heading_offset = rospy.get_param('~heading_offsets', 5.)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 5.)
        self.n_points = rospy.get_param('~number_points', 100)
        self.base_frame = rospy.get_param('~base_frame', "lolo_auv_1/base_link")

        self.nav_goal = None

        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('/global_plan', Path, queue_size=10)
        # rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)

        rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        self._as = actionlib.SimpleActionServer(self._action_name, MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Announced action server with name: %s", self._action_name)
        
        r = rospy.Rate(10) # 10hz
        counter = 0
        while not rospy.is_shutdown():
            if counter % 100 == 0 and self.nav_goal is not None:
               path, pose = self.plan()
               self.pub.publish(path)
            r.sleep()
            counter += 1

if __name__ == '__main__':

    rospy.init_node('bezier_planner')
    planner = BezierPlanner(rospy.get_name())
