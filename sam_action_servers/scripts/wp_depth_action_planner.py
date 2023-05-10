#! /usr/bin/env python

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
#from move_base_msgs.msg import MoveBaseFeedback, MoveBaseResult, MoveBaseAction
from smarc_msgs.msg import GotoWaypointActionFeedback, GotoWaypointResult, GotoWaypointAction, GotoWaypointGoal
import actionlib
import rospy
import tf
from sam_msgs.msg import ThrusterAngles
from smarc_msgs.msg import ThrusterRPM
from std_msgs.msg import Float64, Header, Bool
from std_srvs.srv import SetBool
import math
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from toggle_controller import ToggleController  
import time   

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class ReconfigServer(object):
    def __init__(self, planner):
        """
        Read the defaults from the config object
        so that we dont change the launch-defaults right away
        with different reconfig defaults
        We then just put whatever reconfig we get into the BB with the same key
        """
        # DynamicDynamicReConfig
        # because the .cfg way of doing this is pain
        self.planner = planner #store a copy of the reference to the object
        self.ddrc = DDynamicReconfigure("actions_reconfig")
        # name, description, default value, min, max, edit_method
        self.ddrc.add_variable("lookahead_dist",
                               "Lookahead Distance",
                               planner.lookahead_dist, 1., 20.)


        rospy.loginfo("Started dynamic reconfig server with keys:{}".format(self.ddrc.get_variable_names()))


        # this should be the last thing in this init
        self.ddrc.start(self.reconfig_cb)



    def reconfig_cb(self, config, level):
        for key in self.ddrc.get_variable_names():
            new_value = config.get(key)

            rospy.loginfo("New value for:{} set to:{} )".format(key, new_value))
            self.planner.lookahead_dist = new_value

        return config

class WPDepthPlanner(object):

    # create messages that are used to publish feedback/result
    _feedback = GotoWaypointActionFeedback()
    _result = GotoWaypointResult()
    
    def yaw_feedback_cb(self,yaw_feedback):
        self.yaw_feedback= yaw_feedback.data

    def vel_feedback_cb(self,vel_feedback):
        self.vel_feedback= vel_feedback.data

    def angle_wrap(self,angle):
        if(abs(angle)>3.141516):
            angle= angle - (abs(angle)/angle)*2*3.141516 #Angle wrapping between -pi and pi
            rospy.loginfo_throttle_identical(20, "Angle Error Wrapped")
        return angle

    def turbo_turn(self,angle_error):
        rpm = self.turbo_turn_rpm
        rudder_angle = self.rudder_angle
        flip_rate = self.flip_rate

        left_turn = True
	    #left turn increases value of yaw angle towards pi, right turn decreases it towards -pi.
        if angle_error < 0:
            left_turn = False
            rospy.loginfo('Right turn!')

        rospy.loginfo('Turbo Turning!')
        if left_turn:
            rudder_angle = -rudder_angle

        thrust_rate = 21.
        rate = rospy.Rate(thrust_rate)

        self.vec_pub.publish(0., rudder_angle, Header())
        loop_time = 0.

        rpm1 = ThrusterRPM()
        rpm2 = ThrusterRPM()

        while not rospy.is_shutdown() and loop_time < .37/flip_rate:
            rpm1.rpm = rpm
            rpm2.rpm = rpm
            self.rpm1_pub.publish(rpm1)
            self.rpm2_pub.publish(rpm2)
            loop_time += 1./thrust_rate
            rate.sleep()

        self.vec_pub.publish(0., -rudder_angle, Header())

        loop_time = 0.
        while not rospy.is_shutdown() and loop_time < .63/flip_rate:
            rpm1.rpm = -rpm
            rpm2.rpm = -rpm
            self.rpm1_pub.publish(rpm1)
            self.rpm2_pub.publish(rpm1)
            loop_time += 1./thrust_rate
            rate.sleep()

    def publish_depth_setpoint(self,depth_setpoint):
        #Diving logic to use VBS at low speeds below 0.5 m/s
        if np.abs(self.vel_feedback)< 0.5 and self.vbs_diving_flag:
            #rospy.loginfo_throttle_identical(5, "using VBS")
            self.toggle_depth_ctrl.toggle(True)
            self.toggle_vbs_ctrl.toggle(True)
            #self.vbs_pub.publish(depth_setpoint)
            self.depth_pub.publish(depth_setpoint)
        else:
            #rospy.loginfo_throttle_identical(5, "using DDepth")
            self.toggle_depth_ctrl.toggle(True)
            self.toggle_vbs_ctrl.toggle(False)
            self.depth_pub.publish(depth_setpoint)

    
    def vel_wp_following(self,travel_speed, yaw_setpoint):
        #if speed control is activated from neptus
        #if goal.speed_control_mode == 2:
        rospy.loginfo_throttle_identical(5, "Neptus vel ctrl")
        #with Velocity control
        self.toggle_yaw_ctrl.toggle(True)
        self.yaw_pub.publish(yaw_setpoint)
                
        # Publish to velocity controller
        self.toggle_speed_ctrl.toggle(True)
        #self.vel_pub.publish(self.vel_setpoint)
        self.vel_pub.publish(travel_speed)
        self.toggle_roll_ctrl.toggle(True)
        self.roll_pub.publish(self.roll_setpoint)
        #rospy.loginfo("Velocity published")

    def rpm_wp_following(self,forward_rpm, yaw_setpoint):
        rospy.loginfo_throttle_identical(5,"Using Constant RPM")
        #rospy.loginfo("Using Constant RPM")
        #normal turning if the deviation is small
        self.toggle_vbs_ctrl.toggle(False)
        self.toggle_depth_ctrl.toggle(True)
        
        # Bypass yaw ctrl for now
        # self.toggle_yaw_ctrl.toggle(True)
        # self.yaw_pub.publish(yaw_setpoint)
        # self.toggle_speed_ctrl.toggle(False)
        thrust_ang = ThrusterAngles()
        thrust_ang.header.stamp = rospy.Time.now()
        thrust_ang.thruster_horizontal_radians = yaw_setpoint
        thrust_ang.thruster_vertical_radians = 0.
        self.vec_pub.publish(thrust_ang)
        # TODO: problem with this is the interference between this msg and the depth_ctrl in the vertical thruster

        # Thruster forward
        rpm1 = ThrusterRPM()
        rpm2 = ThrusterRPM()
        rpm1.rpm = int(forward_rpm)
        rpm2.rpm = int(forward_rpm)
        self.rpm_enable_pub.publish(True)
        self.rpm1_pub.publish(rpm1)
        self.rpm2_pub.publish(rpm2)
        #rospy.loginfo("Thrusters forward")
    
    def timer_callback(self, event):
        
        if self.nav_goal is None:
            rospy.loginfo_throttle(30, "Nav goal is None!")
            return

        # Check if the goal has been reached
        #try:

        goal_point = PointStamped()
        goal_point.header.frame_id = self.nav_goal_frame
        goal_point.header.stamp = rospy.Time(0)
        goal_point.point.x = self.nav_goal.waypoint.pose.pose.position.x
        goal_point.point.y = self.nav_goal.waypoint.pose.pose.position.y
        goal_point.point.z = self.nav_goal.waypoint.pose.pose.position.z
        try:
            goal_point_local = self.listener.transformPoint(self.base_frame_2d, goal_point)
            wp_pos = np.array([goal_point_local.point.x, goal_point_local.point.y])
            rospy.loginfo("Dist to WP " + str(np.linalg.norm(wp_pos)))
            
            # Goal reached
            if np.linalg.norm(wp_pos) < self.wp_tolerance:
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self.nav_goal = None
                self._result.reached_waypoint= True           #print("WP control: checking goal ", self.nav_goal.position)
                self.disengage_actuators()
                self._as.set_succeeded(self._result, "WP Reached")
                return

            # Preempted
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.nav_goal = None
                self.disengage_actuators()
                self._as.set_preempted(self._result, "Preempted WP action")
                return
            
            # Otherwise, keep moving

            # Publish setpoints to controllers
            # call function that uses vbs at low speeds, dynamic depth at higher speeds
            self.publish_depth_setpoint(
                self.nav_goal.waypoint.pose.pose.position.z)

            # Current yaw error on local coordinates
            yaw_error = math.atan2(
                goal_point_local.point.y, goal_point_local.point.x)
            rospy.loginfo('Current heading error ' + str(yaw_error))

            # Inverst signs to actuate thrusters
            sign = np.copysign(1, yaw_error)
            yaw_error = -1 * sign * min(self.rudder_angle, abs(yaw_error))

            # Yaw error with tolerance: use the 2/3 of the WP tolerance to create a circle around the WP.
            # If the yaw_error falls within the circle, meaning "abs(yaw_error) > abs(yaw_error_tol)", do not correct.
            yaw_tolerance = self.wp_tolerance * (2./3.)
            d_b = np.sqrt(np.power(np.linalg.norm(
                np.array([goal_point_local.point.x, goal_point_local.point.y])), 2)
                - np.power(yaw_tolerance, 2))

            yaw_error_tol = math.atan2(yaw_tolerance, d_b)
            rospy.loginfo(
                'Current heading error tolerance ' + str(yaw_error_tol))

            yaw_setpoint = yaw_error if abs(
                yaw_error) > abs(yaw_error_tol) else 0.

            if self.vel_ctrl_flag:
                # print("Doing stuff")
                self.vel_wp_following(self.nav_goal.waypoint.travel_speed, yaw_setpoint)
                # self.vel_wp_following(0., yaw_setpoint)
            else:
                # print("Doing stuff")
                self.rpm_wp_following(self.forward_rpm, yaw_setpoint)
                # self.rpm_wp_following(0., yaw_setpoint)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print ("Heading controller: Could not transform WP to base_link")
            pass

 

    def disengage_actuators(self):
        #Stop controllers
        self.toggle_yaw_ctrl.toggle(False)
        self.toggle_depth_ctrl.toggle(False)
        self.toggle_vbs_ctrl.toggle(False)
        self.toggle_roll_ctrl.toggle(False)
        self.toggle_speed_ctrl.toggle(False)

        # Stop thrusters
        self.vel_pub.publish(0.0)
        rpm1 = ThrusterRPM()
        rpm2 = ThrusterRPM()
        rpm1.rpm = 0
        rpm2.rpm = 0
        self.rpm1_pub.publish(rpm1)
        self.rpm2_pub.publish(rpm2)
        self.rpm_enable_pub.publish(False)
        
        
    
    def execute_cb(self, goal):

        rospy.loginfo("Goal received")
        rospy.loginfo(goal)

        #success = True
        self.nav_goal = goal
        if self.nav_goal.waypoint.pose.header.frame_id is None or self.nav_goal.waypoint.pose.header.frame_id == '':
            rospy.logwarn("Goal has no frame id! Using utm by default")
            self.nav_goal.waypoint.pose.header.frame_id = 'utm'  # 'utm'

        # self.nav_goal.position.z = goal.waypoint.travel_depth # assign waypoint depth from neptus, goal.z is 0.
        if goal.waypoint.speed_control_mode == 2: #GotoWaypointGoal.SPEED_CONTROL_SPEED: #2:
            self.vel_ctrl_flag = True # check if NEPTUS sets a velocity
        elif goal.waypoint.speed_control_mode == 1: #GotoWaypointGoal.SPEED_CONTROL_RPM: # 1: 
            self.vel_ctrl_flag = False # use RPM ctrl
            self.forward_rpm = goal.waypoint.travel_rpm #take rpm from NEPTUS

        #get wp goal tolerance from Neptus
        if goal.waypoint.goal_tolerance:
            self.wp_tolerance = goal.waypoint.goal_tolerance #take the goal tolerance from Neptus if it exists!
            rospy.loginfo_throttle(5,'Using Goal tolerance from UI:'+ str(goal.waypoint.goal_tolerance))

        if self.use_constant_rpm: #overriding neptus values
            self.vel_ctrl_flag = False #use constant rpm



    def __init__(self, name):

        """Publish yaw and depth setpoints based on waypoints"""
        self._action_name = name

        #self.heading_offset = rospy.get_param('~heading_offsets', 5.)
        self.wp_tolerance = rospy.get_param('~wp_tolerance', 5.) #default value, overriden by Neptus if it is set in Neptus
        self.depth_tolerance = rospy.get_param('~depth_tolerance', 0.5)

        self.base_frame = rospy.get_param('~base_frame', "sam/base_link")
        self.base_frame_2d = rospy.get_param('~base_frame_2d', "sam/base_link")

        rpm1_cmd_topic = rospy.get_param('~rpm1_cmd_topic', '/sam/core/thruster1_cmd')
        rpm2_cmd_topic = rospy.get_param('~rpm2_cmd_topic', '/sam/core/thruster2_cmd')
        rpm_enable_topic = rospy.get_param('~rpm_enable_topic', '/sam/ctrl/goto_waypoint/rpm/enable')
        heading_setpoint_topic = rospy.get_param('~heading_setpoint_topic', '/sam/ctrl/dynamic_heading/setpoint')
        depth_setpoint_topic = rospy.get_param('~depth_setpoint_topic', '/sam/ctrl/dynamic_depth/setpoint')

        self.use_constant_rpm =  rospy.get_param('~use_constant_rpm', False)
        self.forward_rpm = int(rospy.get_param('~forward_rpm', 1000))
        
        #augmentation for vbs diving at low speeds, crosstrack error and waypoint overshoot
        self.vbs_diving_flag = rospy.get_param('~vbs_diving_flag', True)
        self.crosstrack_flag = rospy.get_param('~crosstrack_flag', True)
        self.lookahead_dist = rospy.get_param('~lookahead_dist', 3.0)
        self.wp_overshoot_flag =  rospy.get_param('~wp_overshoot_flag', True)
        self.timeout_flag = rospy.get_param('~timeout_flag', True)
        self.timeout_limit = rospy.get_param('~timeout_limit', 500)

        #related to turbo turn
        self.turbo_turn_flag = rospy.get_param('~turbo_turn_flag', False)
        thrust_vector_cmd_topic = rospy.get_param('~thrust_vector_cmd_topic', '/sam/core/thrust_vector_cmd')
        yaw_feedback_topic = rospy.get_param('~yaw_feedback_topic', '/sam/ctrl/yaw_feedback')
        self.turbo_angle_min_deg = rospy.get_param('~turbo_angle_min', 90)
        self.turbo_angle_min = np.radians(self.turbo_angle_min_deg)
        self.turbo_angle_max = 3.0
        self.flip_rate = rospy.get_param('~flip_rate', 0.5)
        self.rudder_angle = rospy.get_param('~rudder_angle', 0.08)
        self.turbo_turn_rpm = rospy.get_param('~turbo_turn_rpm', 1000)
        vbs_setpoint_topic = rospy.get_param('~vbs_setpoint_topic', '/sam/ctrl/vbs/setpoint')


	#related to velocity regulation instead of rpm
        self.vel_ctrl_flag = rospy.get_param('~vel_ctrl_flag', False)
        #self.vel_setpoint = rospy.get_param('~vel_setpoint', 0.5) #velocity setpoint in m/s
        self.roll_setpoint = rospy.get_param('~roll_setpoint', 0)
        vel_setpoint_topic = rospy.get_param('~vel_setpoint_topic', '/sam/ctrl/dynamic_velocity/u_setpoint')
        roll_setpoint_topic = rospy.get_param('~roll_setpoint_topic', '/sam/ctrl/dynamic_velocity/roll_setpoint')
        vel_feedback_topic = rospy.get_param('~vel_feedback_topic', '/sam/dr/u_feedback')

        #controller services
        toggle_yaw_ctrl_service = rospy.get_param('~toggle_yaw_ctrl_service', '/sam/ctrl/toggle_yaw_ctrl')
        toggle_depth_ctrl_service = rospy.get_param('~toggle_depth_ctrl_service', '/sam/ctrl/toggle_depth_ctrl')
        toggle_vbs_ctrl_service = rospy.get_param('~toggle_vbs_ctrl_service', '/sam/ctrl/toggle_vbs_ctrl')
        toggle_speed_ctrl_service = rospy.get_param('~toggle_speed_ctrl_service', '/sam/ctrl/toggle_speed_ctrl')
        toggle_roll_ctrl_service = rospy.get_param('~toggle_roll_ctrl_service', '/sam/ctrl/toggle_roll_ctrl')
        self.toggle_yaw_ctrl = ToggleController(toggle_yaw_ctrl_service, False)
        self.toggle_depth_ctrl = ToggleController(toggle_depth_ctrl_service, False)
        self.toggle_vbs_ctrl = ToggleController(toggle_vbs_ctrl_service, False)
        self.toggle_speed_ctrl = ToggleController(toggle_speed_ctrl_service, False)
        self.toggle_roll_ctrl = ToggleController(toggle_roll_ctrl_service, False)

        #initializing some global variables
        self.nav_goal = None
        self.x_prev = 0
        self.y_prev = 0
        self.prev_xydiff_norm = 0
        self.error_gradient = 0

        self.listener = tf.TransformListener()
        #rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        self.yaw_feedback = 0.0
        rospy.Subscriber(yaw_feedback_topic, Float64, self.yaw_feedback_cb)
        self.vel_feedback = 0.0
        rospy.Subscriber(vel_feedback_topic, Float64, self.vel_feedback_cb)

        self.rpm1_pub = rospy.Publisher(rpm1_cmd_topic, ThrusterRPM, queue_size=10)
        self.rpm2_pub = rospy.Publisher(rpm2_cmd_topic, ThrusterRPM, queue_size=10)
        self.rpm_enable_pub = rospy.Publisher(rpm_enable_topic, Bool, queue_size=10)
        self.yaw_pub = rospy.Publisher(heading_setpoint_topic, Float64, queue_size=10)
        self.depth_pub = rospy.Publisher(depth_setpoint_topic, Float64, queue_size=10)
        self.vel_pub = rospy.Publisher(vel_setpoint_topic, Float64, queue_size=10)
        self.roll_pub = rospy.Publisher(roll_setpoint_topic, Float64, queue_size=10)

        #TODO make proper if it works.
        self.vbs_pub = rospy.Publisher(vbs_setpoint_topic, Float64, queue_size=10)
        self.vec_pub = rospy.Publisher(thrust_vector_cmd_topic, ThrusterAngles, queue_size=10)

        self._as = actionlib.SimpleActionServer(self._action_name, GotoWaypointAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Announced action server with name: %s", self._action_name)
        
        rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        reconfig = ReconfigServer(self)


        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('wp_depth_action_planner')
    planner = WPDepthPlanner(rospy.get_name())
    
