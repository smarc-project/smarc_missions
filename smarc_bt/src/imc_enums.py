#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

########################
# IMC Enums
########################
STATE_BLOCKED = 0
STATE_READY = 1
STATE_INITIALIZING = 2
STATE_EXECUTING = 3

OP_MODE_SERVICE = 0
OP_MODE_CALIBRATION = 1
OP_MODE_ERROR = 2
OP_MODE_MANEUVER = 3
OP_MODE_EXTERNAL = 4
OP_MODE_BOOT = 5

PLANDB_TYPE_REQUEST = 0
PLANDB_TYPE_SUCCESS = 1
PLANDB_TYPE_FAILURE = 2
PLANDB_TYPE_IN_PROGRESS = 3
PLANDB_OP_SET = 0
PLANDB_OP_DEL = 1
PLANDB_OP_GET = 2
PLANDB_OP_GET_INFO = 3
PLANDB_OP_CLEAR = 4
PLANDB_OP_GET_STATE = 5
PLANDB_OP_GET_DSTATE = 6
PLANDB_OP_BOOT = 7

# TODO eventually implement other types of maneuvers
MANEUVER_GOTO = 450

# a list of actions that we will consider
# as the imc 'executing' state
EXECUTING_ACTION_NAMES =[
    'A_GotoWaypoint',
    'A_SetNextPlanAction' #so we dont spam service/maneuver when going tru a lot of waypoints quickly
]
# same thing for the 'blocked' state
BLOCKED_ACTION_NAMES =[
    'A_EmergencySurface'
]
