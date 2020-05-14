#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)




# in ms.
BT_TICKING_PERIOD = 50

##########################
# ROS THINGS
# THESE SHOULD BE RECEIVED FROM ROS_PARAMS AND SET IN THE MAIN FUNCTION
# SET THEM AS sam_globals.THING and use as sam_globals.THING ELSEWHERE
##########################
# DO NOT PUT SLASHES BEFORE TOPIC NAMES, that puts them in 'root' in ros's eyes.

# topics blah
DEPTH_TOPIC = 'ctrl/depth_feedback'
ALTITUDE_TOPIC = 'ctrl/altitude_feedback'
LEAK_TOPIC = 'core/leak_fb'
GPS_FIX_TOPIC = 'core/gps'

BASE_LINK = 'base_link'

PLAN_TOPIC = 'imc/plan_db'
PLAN_CONTROL_TOPIC = 'imc/plan_control'
ESTIMATED_STATE_TOPIC = 'imc/estimated_state'
PLAN_CONTROL_STATE_TOPIC = 'imc/plan_control_state'
VEHICLE_STATE_TOPIC = 'imc/vehicle_state'
ABORT_TOPIC = 'imc/abort'

ACTION_NAMESPACE = 'ctrl/wp_depth_action_planner'
EMERGENCY_ACTION_NAMESPACE = 'ctrl/emergency_surface_action'


######################
# BLACKBOARD VARIABLES
######################
ABORT_BB = 'abort'
LEAK_BB = 'leak'

DEPTH_BB = 'depth'
ALTITUDE_BB = 'altitude'

MISSION_PLAN_MSG_BB = 'plan_msg'
MISSION_PLAN_OBJ_BB = 'misison_plan'

UTM_BAND_BB = 'utm_band'
UTM_ZONE_BB = 'utm_zone'

WORLD_ROT_BB = 'world_rot'
WORLD_TRANS_BB = 'world_trans'
BASE_LINK_BB = 'base_link'

IMC_STATE_BB = 'imc_state'

CURRENT_PLAN_ACTION = 'current_plan_action'
LAST_PLAN_ACTION_FEEDBACK = 'last_plan_action_feedback'
# set this from any action that might return RUNNING.
# useful for feedback purposes
CURRENTLY_RUNNING_ACTION = 'currently_running_action'


PLAN_TOPIC_BB = 'plan_topic'
PLAN_CONTROL_MSG_BB = 'plan_control'
ESTIMATED_STATE_TOPIC_BB = 'estimated_state'
PLAN_CONTROL_STATE_TOPIC_BB = 'plan_ctrl_state'
VEHICLE_STATE_TOPIC_BB = 'vehicle_state'

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

IMC_PLANDB_TYPE_REQUEST = 0
IMC_PLANDB_TYPE_SUCCESS = 1
IMC_PLANDB_TYPE_FAILURE = 2
IMC_PLANDB_TYPE_IN_PROGRESS = 3
IMC_PLANDB_OP_SET = 0
IMC_PLANDB_OP_DEL = 1
IMC_PLANDB_OP_GET = 2
IMC_PLANDB_OP_GET_INFO = 3
IMC_PLANDB_OP_CLEAR = 4
IMC_PLANDB_OP_GET_STATE = 5
IMC_PLANDB_OP_GET_DSTATE = 6
IMC_PLANDB_OP_BOOT = 7

# TODO eventually implement other types of maneuvers
IMC_MANEUVER_GOTO = 450




########################
# DEFAULT VALUES
########################
SAM_MAX_DEPTH = 5
SAM_MIN_ALTITUDE = 2

# these are from croatia, biograd coast
DEFAULT_UTM_ZONE = 33
DEFAULT_UTM_BAND = 'T'
# if set to true, utm zone and band will be set from
# the gps fix we read instead of rosparams
TRUST_GPS = True
