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
import tf

import geometry as geom

class SimpleRPMGoal(object):
    def __init__(self,
                 x,
                 y,
                 depth,
                 rpm,
                 tolerance):
        self.x = x
        self.y = y
        self.depth = depth
        self.rpm = rpm
        self.tolerance = tolerance

    @property
    def pos(self):
        return np.array([self.x, self.y, self.depth])



class Lolo(object):
    IDLE = "IDLE"
    DRIVE = "DRIVE"
    THRUSTER_TURN = "THRUSTER_TURN"
    BACKWARDS_SPIRAL = "BACKWARDS_SPIRAL"

    def __init__(self,
                 max_rpm = 2000,
                 max_fin_radians = 0.6,
                 rudder_Kp = 50,
                 elevator_Kp = 50,
                 rudder_cone_degrees = 5.72,
                 forward_cone_degrees = 10,
                 enable_spiral = False,
                 enable_thruster_turn = True):
        """
        A container object that abstracts away ros-related stuff for a nice abstract vehicle
        pose is in NED, x = north, y = east, z = down/depth
        """

        self.max_rpm = max_rpm
        self.max_fin_radians = max_fin_radians
        self.rudder_Kp = rudder_Kp
        self.elevator_Kp = elevator_Kp
        self.rudder_cone_degrees = rudder_cone_degrees
        self.forward_cone_degrees = forward_cone_degrees
        self.enable_spiral = enable_spiral
        self.enable_thruster_turn = enable_thruster_turn

        self.goal = None
        self.control_mode = Lolo.IDLE

        self.pos = np.zeros(3)
        self.ori_rpy = np.zeros(3)

        self.thruster_rpms = np.zeros(2)
        self.desired_rpms = np.zeros(2)

        self.elevon_angles = np.zeros(2)
        self.desired_elevon_angles = np.zeros(2)

        self.rudder_angle = 0
        self.desired_rudder_angle = 0

        self.elevator_angle = 0
        self.desired_elevator_angle = 0

    def _reset_desires(self):
        print("Reset desires to 0")
        self.desired_elevator_angle = 0
        self.desired_rudder_angle = 0
        self.desired_elevon_angles[0] = 0
        self.desired_elevon_angles[1] = 0
        self.desired_rpms[0] = 0
        self.desired_rpms[1] = 0

    def _change_mode(self, new_mode):
        if new_mode == self.control_mode:
            return
        print("Changing mode: {} -> {}".format(self.control_mode, new_mode))
        self._reset_desires()
        self.control_mode = new_mode

    ######################################
    # Call this when you want lolo to actually control something
    ######################################
    def update(self):
        if self.goal is None:
            self._change_mode(Lolo.IDLE)
            return

        ####
        # Get all the diffs towards the goal
        ####
        depth_diff = self.goal.depth - self.depth
        pitch_diff = np.arctan2(depth_diff, self.xy_dist_to_goal) * geom.RADTODEG
        xy_diff = self.position_error[:2]
        yaw_diff = geom.vec2_directed_angle(self.yaw_vec, xy_diff) * geom.RADTODEG

        ####
        # Change mode according to where the goal is relative to us
        ####
        # depth control priority
        if np.abs(pitch_diff) > self.forward_cone_degrees and np.abs(depth_diff) > self.goal.tolerance:
            if self.enable_spiral:
                self._change_mode(Lolo.BACKWARDS_SPIRAL)
            else:
                self._change_mode(Lolo.DRIVE)
        # thruster-turn second priority
        elif np.abs(yaw_diff) > self.rudder_cone_degrees:
            if self.enable_thruster_turn:
                self._change_mode(Lolo.THRUSTER_TURN)
            else:
                self._change_mode(Lolo.DRIVE)
        # no special mode needed, just drive to goal
        else:
            self._change_mode(Lolo.DRIVE)


        ####
        # Finally control depending on the mode
        ####
        if self.control_mode == Lolo.IDLE:
            self._reset_desires()
            return

        # Save some compute by doing this here
        # 1 = go down, -1 = go up 
        pitch_direction = np.sign(pitch_diff)
        pitch_mag = np.abs(pitch_diff)/180.

        if self.control_mode == Lolo.BACKWARDS_SPIRAL:
            self._backwards_spiral_to_depth(pitch_direction)
            return

        # Save some compute by doing this here
        # 1=turn right -1=turn left
        turn_direction = np.sign(yaw_diff)
        # map 0-180 to 0-1
        turn_mag = np.abs(yaw_diff)/180.

        if self.control_mode == Lolo.THRUSTER_TURN:
            self._thruster_turn_to_yaw(turn_direction, turn_mag)
            return


        if self.control_mode == Lolo.DRIVE:
            self._drive_to_yaw(turn_direction, turn_mag)
            self._drive_to_depth(pitch_direction, pitch_mag)
            return


    def _backwards_spiral_to_depth(self, pitch_direction):
        self.desired_elevator_angle = -pitch_direction * self.max_fin_radians
        self.desired_rpms[0] = -self.max_rpm
        self.desired_rpms[1] = -self.max_rpm

        # to make a spiral, turn as harshly as we can
        self.desired_rudder_angle = self.max_fin_radians
        self.desired_elevon_angles[0] = self.max_fin_radians
        self.desired_elevon_angles[1] = self.max_fin_radians

        # and also if deep enough, use the elevons to pitch moar
        if self.depth > 1:
            self.desired_elevon_angles[0] = self.desired_elevator_angle
            self.desired_elevon_angles[1] = self.desired_elevator_angle
        else:
            self.desired_elevon_angles[0] = 0
            self.desired_elevon_angles[1] = 0


    def _drive_to_depth(self, pitch_direction, pitch_mag):
        self.desired_elevator_angle = -pitch_direction * pitch_mag * self.max_fin_radians * self.elevator_Kp
        # TODO extract that 1m into config?
        # we dont wanna use the elevons when shallow
        # otherwise the thursters just lift up the water and we dont dive at all
        if self.depth > 1:
            self.desired_elevon_angles[0] = self.desired_elevator_angle
            self.desired_elevon_angles[1] = self.desired_elevator_angle
        else:
            self.desired_elevon_angles[0] = 0
            self.desired_elevon_angles[1] = 0


    def _thruster_turn_to_yaw(self, turn_direction, turn_mag):
        self.desired_rudder_angle = turn_direction * turn_mag * self.max_fin_radians * self.rudder_Kp
        self.desired_rpms[0] = -turn_direction * self.max_rpm
        self.desired_rpms[1] =  turn_direction * self.max_rpm


    def _drive_to_yaw(self, turn_direction, turn_mag):
        """
        Simple P controller that also uses
        the thrusters to do some tight turns when needed
        """
        # super simple P controller for rudder
        self.desired_rudder_angle = turn_direction * turn_mag * self.max_fin_radians * self.rudder_Kp
        # and just drive forward
        # TODO possibly tune these for some light P control too
        self.desired_rpms[0] = self.goal.rpm
        self.desired_rpms[1] = self.goal.rpm



    ###############################
    ### Outward facing stuff, mostly automated away from this object
    ###############################
    def set_goal(self,x,y,depth,rpm,tolerance):
        self.goal = SimpleRPMGoal(x,y,depth,rpm,tolerance)

    def reset_goal(self):
        self.goal = None
        self.update()

    def update_pos(self, x=None, y=None, depth=None):
        if x is not None: self.pos[0] = x
        if y is not None: self.pos[1] = y
        if depth: self.pos[2] = depth

    def update_ori(self, r=None, p=None, y=None):
        if r is not None: self.ori_rpy[0] = r
        if p is not None: self.ori_rpy[1] = p
        if y is not None: self.ori_rpy[2] = y

    def update_thruster_rpms(self, port=None, strb=None):
        if port is not None: self.thruster_rpms[0] = port
        if strb is not None: self.thruster_rpms[1] = strb

    def update_elevator_angle(self, a):
        self.elevator_angle = a

    def update_elevon_angles(self, port=None, strb=None):
        if port is not None: self.elevon_angles[0] = port
        if strb is not None: self.elevon_angles[1] = strb

    def update_rudder_angle(self, a):
        self.rudder_angle = a

    def update_elevator_angle(self, a):
        self.elevator_angle = a

    def blarg(self):
        self.desired_elevator_angle = np.random.standard_normal()*0.6
        self.desired_rudder_angle = np.random.standard_normal()*0.6
        self.desired_elevon_angles[0] = np.random.standard_normal()*0.6
        self.desired_elevon_angles[1] = np.random.standard_normal()*0.6
        self.desired_rpms[0] = np.random.standard_normal()*2000
        self.desired_rpms[1] = np.random.standard_normal()*2000


    ###############################
    ### Properties for convenience
    ###############################
    @property
    def x(self):
        return self.pos[0]
    @property
    def y(self):
        return self.pos[1]
    @property
    def depth(self):
        return self.pos[2]
    @property
    def roll(self):
        return self.ori_rpy[0]
    @property
    def pitch(self):
        return self.ori_rpy[1]
    @property
    def yaw(self):
        return self.ori_rpy[2]
    @property
    def yaw_vec(self):
        return np.array([np.cos(self.yaw), np.sin(self.yaw)])
    @property
    def ori_quat(self):
        return tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
    @property
    def port_rpm(self):
        return self.thruster_rpms[0]
    @property
    def strb_rpm(self):
        return self.thruster_rpms[1]
    @property
    def port_elevon_angle(self):
        return self.elevon_angles[0]
    @property
    def strb_elevon_angle(self):
        return self.elevon_angles[1]
    @property
    def position_error(self):
        return self.goal.pos - self.pos
    @property
    def xy_dist_to_goal(self):
        return geom.euclid_distance(self.goal.pos[:2], self.pos[:2])
    @property
    def depth_to_goal(self):
        return self.goal.depth - self.depth






