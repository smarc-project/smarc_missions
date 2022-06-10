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

    def rpm_wp_following(self,forward_rpm,yaw_setpoint):
        rospy.loginfo_throttle_identical(5,"Using Constant RPM")
        #rospy.loginfo("Using Constant RPM")
        #normal turning if the deviation is small
        self.toggle_vbs_ctrl.toggle(False)
        self.toggle_depth_ctrl.toggle(True)
        self.toggle_yaw_ctrl.toggle(True)
        self.yaw_pub.publish(yaw_setpoint)
        self.toggle_speed_ctrl.toggle(False)
        # Thruster forward
        rpm1 = ThrusterRPM()
        rpm2 = ThrusterRPM()
        rpm1.rpm = int(forward_rpm)
        rpm2.rpm = int(forward_rpm)
        self.rpm_enable_pub.publish(True)
        self.rpm1_pub.publish(rpm1)
        self.rpm2_pub.publish(rpm2)
        #rospy.loginfo("Thrusters forward")

    def check_success(self, position_feedback, nav_goal):
        start_pos = np.array(position_feedback)
        end_pos = np.array([nav_goal.position.x, nav_goal.position.y, nav_goal.position.z])

        # We check for success out of the main control loop in case the main control loop is
        # running at 300Hz or sth. like that. We dont need to check succes that frequently.
        xydiff = start_pos[:2] - end_pos[:2]
        zdiff = np.abs(np.abs(start_pos[2]) - np.abs(end_pos[2]))
        xydiff_norm = np.linalg.norm(xydiff)

        #See if error is decreasing or increasing
        self.error_gradient = xydiff_norm - self.prev_xydiff_norm
        self.prev_xydiff_norm = xydiff_norm
        # rospy.logdebug("diff xy:"+ str(xydiff_norm)+' z:' + str(zdiff))
        #rospy.loginfo_throttle_identical(5, "Using Crosstrack Error")
        rospy.loginfo_throttle_identical(5, "diff xy:"+ str(xydiff_norm)+' z:' + str(zdiff)+ " WP tol:"+ str(self.wp_tolerance)+ "Depth tol:"+str(self.depth_tolerance))
        if xydiff_norm < self.wp_tolerance and zdiff < self.depth_tolerance:
            rospy.loginfo("Reached WP!")
            self.x_prev = self.nav_goal.position.x
            self.y_prev = self.nav_goal.position.y
            self.nav_goal = None
            self._result.reached_waypoint= True
            #self._as.set_succeeded(self._result, "Reached WP")

        #consider timeout
        current_time = time.time()
        wp_duration = current_time-self.start_time
        if xydiff_norm < 20 and wp_duration > self.timeout_limit:
            rospy.loginfo("Timeout, going to next WP!")
            self.nav_goal = None
            self._result.reached_waypoint= True
        
        self.wp_distance = xydiff_norm

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
        
        
    
    def execute_cb(self, goal):

        rospy.loginfo("Goal received")
        rospy.loginfo(goal)
        self.start_time = time.time()

        #success = True
        self.nav_goal = goal.waypoint.pose.pose
        self.nav_goal_frame = goal.waypoint.pose.header.frame_id
        if self.nav_goal_frame is None or self.nav_goal_frame == '':
            rospy.logwarn("Goal has no frame id! Using utm by default")
            self.nav_goal_frame = 'utm' #'utm'

        self.nav_goal.position.z = goal.waypoint.travel_depth # assign waypoint depth from neptus, goal.z is 0.
        if goal.waypoint.speed_control_mode == 2: #GotoWaypointGoal.SPEED_CONTROL_SPEED: #2:
            self.vel_ctrl_flag = True # check if NEPTUS sets a velocity
        elif goal.waypoint.speed_control_mode == 1: #GotoWaypointGoal.SPEED_CONTROL_RPM: # 1: 
            self.vel_ctrl_flag = False # use RPM ctrl
            self.forward_rpm = goal.waypoint.travel_rpm #take rpm from NEPTUS

        #get wp goal tolerance from Neptus
        if goal.waypoint.goal_tolerance:
            self.wp_tolerance = goal.waypoint.goal_tolerance #take the goal tolerance from Neptus if it exists!
            rospy.loginfo_throttle(5,'Using Goal tolerance from neptus:'+ str(goal.waypoint.goal_tolerance))

        if self.use_constant_rpm: #overriding neptus values
            self.vel_ctrl_flag = False #use constant rpm


        goal_point = PointStamped()
        goal_point.header.frame_id = self.nav_goal_frame
        goal_point.header.stamp = rospy.Time(0)
        goal_point.point.x = self.nav_goal.position.x
        goal_point.point.y = self.nav_goal.position.y
        goal_point.point.z = self.nav_goal.position.z
        try:
            goal_point_local = self.listener.transformPoint(self.nav_goal_frame, goal_point)
            self.nav_goal.position.x = goal_point_local.point.x
            self.nav_goal.position.y = goal_point_local.point.y
            self.nav_goal.position.z = goal_point_local.point.z
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print ("Not transforming point to world local")
            pass

        rospy.loginfo('Nav goal in local %s ' % self.nav_goal.position.x)

        r = rospy.Rate(20.) # 10hz
        counter = 0
        while not rospy.is_shutdown() and self.nav_goal is not None:
            
            # Preempted
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                #success = False
                self.nav_goal = None
                self.disengage_actuators()

                print('wp depth action planner: stopped thrusters')
                self._as.set_preempted(self._result, "Preempted WP action")
                return

            # Compute controller setpoints
            if counter % 5 == 0:
                try:
                    (trans, rot) = self.listener.lookupTransform(self.nav_goal_frame, self.base_frame, rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("Error with tf:"+str(self.nav_goal_frame) + " to "+str(self.base_frame))
                    continue

                pose_fb = PoseStamped()
                pose_fb.header.frame_id = self.nav_goal_frame
                pose_fb.pose.position.x = trans[0]
                pose_fb.pose.position.y = trans[1]
                pose_fb.pose.position.z = trans[2]
                #self._feedback.feedback.pose = pose_fb
                #self._feedback.feedback.pose.header.stamp = rospy.get_rostime()
                #self._as.publish_feedback(self._feedback)
                #rospy.loginfo("Sending feedback")
                
                xdiff = self.nav_goal.position.x - pose_fb.pose.position.x
                ydiff = self.nav_goal.position.y - pose_fb.pose.position.y

                '''#Compensate for overshooting a waypoint
                if not self.wp_overshoot_flag:
                    self.error_gradient = -1 #Disable overshoot compensation
                
                overshot_wp = False
                if self.error_gradient > 0:
                    overshot_wp = True #logic for overshooting waypoint'''

                '''if overshot_wp:
                    rospy.loginfo_throttle_identical(5, "Compensating for overshoot!")
                    self.crosstrack_flag = False
                else:
                    self.crosstrack_flag = False'''
                
                #self.crosstrack_flag = False
                if self.crosstrack_flag:
                    #considering cross-track error according to Fossen, Page 261 eq 10.73,10.74
                    x_goal = self.nav_goal.position.x
                    y_goal = self.nav_goal.position.y

                    #checking if there is a previous WP, if there is no previous WP, it considers the current position. 
                    # It also uses this check to see if we overshot the WP and compensates for overshoot
                    if not self.wp_overshoot_flag:
                        self.error_gradient = -1 #Disable overshoot compensation
                    
                    overshot_wp = False
                    if self.error_gradient > 0:
                        overshot_wp = True #logic for overshooting waypoint

                    if (self.y_prev == 0 and self.x_prev == 0) or overshot_wp:
                        rospy.loginfo_throttle_identical(5, "Compensating for overshoot!")
                        self.y_prev = pose_fb.pose.position.y
                        self.x_prev = pose_fb.pose.position.x
                
                    y_prev = self.y_prev #read previous WP
                    x_prev = self.x_prev 

                    #considering cross-track error according to Fossen, Page 261 eq 10.73,10.74
                    err_tang = math.atan2(y_goal-y_prev, x_goal- x_prev) # path tangential vector
                    #err_tang = math.atan2(ydiff, xdiff) # path tangential vector
                    err_crosstrack = -(pose_fb.pose.position.x - x_prev)*math.sin(err_tang)+ (pose_fb.pose.position.y - y_prev)*math.cos(err_tang) # crosstrack error
                    #lookahead = 3 #lookahead distance(m)
                    lookahead = self.lookahead_dist
                    err_velpath = math.atan2(-err_crosstrack,lookahead)

                    yaw_setpoint = (err_tang) + (err_velpath)
                    rospy.loginfo_throttle_identical(5, "Using Crosstrack Error, err_tang ="+str(err_tang)+"err_velpath"+str(err_velpath))
                
                else:
                    #Compute yaw setpoint based on waypoint position and current position
                    #xdiff = self.nav_goal.position.x - pose_fb.pose.position.x
                    #ydiff = self.nav_goal.position.y - pose_fb.pose.position.y
                    #yaw_setpoint = 1.57-math.atan2(ydiff,xdiff)
                    #The original yaw setpoint!
                    yaw_setpoint = math.atan2(ydiff,xdiff)
                    rospy.loginfo_throttle_identical(5, "Using normal WP following")
                    #print('xdiff:',xdiff,'ydiff:',ydiff,'yaw_setpoint:',yaw_setpoint)

		        #compute yaw_error (e.g. for turbo_turn)
                yaw_error= -(self.yaw_feedback - yaw_setpoint)
                yaw_error= self.angle_wrap(yaw_error) #wrap angle error between -pi and pi

                depth_setpoint = self.nav_goal.position.z
                #depth_setpoint = goal.travel_depth
                #rospy.loginfo("Depth setpoint: %f", depth_setpoint)

            ## Publish setpoints to controllers 
            self.publish_depth_setpoint(depth_setpoint) # call function that uses vbs at low speeds, dynamic depth at higher speeds

            #Not used currently, feature for turboturn when waypoint is close
            '''if self.wp_distance < 8.0: 
                wp_is_close = True
            else:
                wp_is_close = False'''
                

            if self.turbo_turn_flag:
            #if turbo turn is included, turbo turn at large yaw deviations
                if (abs(yaw_error) > self.turbo_angle_min and abs(yaw_error) < self.turbo_angle_max): # or wp_is_close:
                    rospy.loginfo("Yaw error: %f", yaw_error)
                    #turbo turn with large deviations, maximum deviation is 3.0 radians to prevent problems with discontinuities at +/-pi
                    self.toggle_yaw_ctrl.toggle(False)
                    self.toggle_speed_ctrl.toggle(False)
                    self.turbo_turn(yaw_error)
                    self.toggle_depth_ctrl.toggle(False)
                    self.toggle_vbs_ctrl.toggle(True)
                    #self.depth_pub.publish(depth_setpoint) #Already
                else:
                #if it is outside the turboturning range
                    if self.vel_ctrl_flag:
                        self.vel_wp_following(goal.waypoint.travel_speed, yaw_setpoint)
                    else:
                        self.rpm_wp_following(self.forward_rpm, yaw_setpoint)
            else:
                #if it is not turboturning
                if self.vel_ctrl_flag:
                    self.vel_wp_following(goal.waypoint.travel_speed, yaw_setpoint)
                else:
                    self.rpm_wp_following(self.forward_rpm, yaw_setpoint)

            if counter % 10 == 0: 
                #periodically check if waypoint is reached
                self.check_success(trans,self.nav_goal)

            counter += 1
            r.sleep()

        self.disengage_actuators()
        #self.x_prev = self.nav_goal.position.x
        #self.y_prev = self.nav_goal.position.y
        #self._result.reached_waypoint= True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result,"WP Reached")


    def __init__(self, name):

        """Publish yaw and depth setpoints based on waypoints"""
        self._action_name = name

        #self.heading_offset = rospy.get_param('~heading_offsets', 5.)
        self.wp_tolerance = rospy.get_param('~wp_tolerance', 5.) #default value, overriden by Neptus if it is set in Neptus
        self.depth_tolerance = rospy.get_param('~depth_tolerance', 0.5)

        self.base_frame = rospy.get_param('~base_frame', "sam/base_link")

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
        self.start_time = 0
        self.wp_distance = 1000

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

        reconfig = ReconfigServer(self)

        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('wp_depth_action_planner')
    planner = WPDepthPlanner(rospy.get_name())
    
