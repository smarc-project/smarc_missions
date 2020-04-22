#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)




# in ms.
BT_TICKING_PERIOD = 50

##########################
# ROS THINGS
##########################
# DO NOT PUT SLASHES BEFORE TOPIC NAMES, that puts them in 'root' in ros's eyes.

# IMC Bridge connections
PLAN_TOPIC = 'imc/plan_db'
ESTIMATED_STATE_TOPIC = 'imc/estimated_state'
PLAN_CONTROL_STATE_TOPIC = 'imc/plan_control_state'
ABORT_TOPIC = 'imc/abort'
VEHICLE_STATE_TOPIC = 'imc/vehicle_state'

# Controller connections
DEPTH_TOPIC = 'ctrl/depth_feedback'
ALTITUDE_TOPIC = 'ctrl/altitude_feedback'
SAM_VBS_SETPOINT_TOPIC = 'ctrl/vbs/setpoint'
SAM_PID_ENABLE_TOPIC = 'ctrl/vbs/pid_enable'
SAM_VBS_CONTROL_ACTION_TOPIC = 'ctrl/vbs/control_action'

# direct access to vbs command, skipping the controller in between
SAM_VBS_CMD_TOPIC = 'core/vbs_cmd'
LEAK_TOPIC = 'core/leak_fb'

BASE_LINK = '/sam/base_link'
ACTION_NAMESPACE = '/bezier_planner'


######################
# BLACKBOARD VARIABLES
######################
ABORT_BB = 'abort'
LEAK_BB = 'leak'

DEPTH_BB = 'depth'
ALTITUDE_BB = 'altitude'

MISSION_PLAN_STR_BB = 'plan_str'
MISSION_PLAN_OBJ_BB = 'misison_plan'

UTM_BAND_BB = 'utm_band'
UTM_ZONE_BB = 'utm_zone'

WORLD_ROT_BB = 'world_rot'
WORLD_TRANS_BB = 'world_trans'

IMC_STATE_BB = 'imc_state'

CURRENT_PLAN_ACTION = 'current_plan_action'
LAST_PLAN_ACTION_FEEDBACK = 'last_plan_action_feedback'
# set this from any action that might return RUNNING.
# useful for feedback purposes
CURRENTLY_RUNNING_ACTION = 'currently_running_action'


########################
# IMC Enums
########################
IMC_STATE_BLOCKED = 0
IMC_STATE_READY = 1
IMC_STATE_INITIALIZING = 2
IMC_STATE_EXECUTING = 3

IMC_OP_MODE_SERVICE = 0
IMC_OP_MODE_CALIBRATION = 1
IMC_OP_MODE_ERROR = 2
IMC_OP_MODE_MANEUVER = 3
IMC_OP_MODE_EXTERNAL = 4
IMC_OP_MODE_BOOT = 5




########################
# DEFAULT VALUES
########################
SAM_MAX_DEPTH = 5
SAM_MIN_ALTITUDE = 2

MINIMUM_PLAN_STR_LEN = 135 # somehow Chris found this number, i'll reuse this

# these are from croatia, biograd coast
DEFAULT_UTM_ZONE = 33
DEFAULT_UTM_BAND = 'T'
