#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

########################
# IMC Enums
# See https://github.com/LSTS/imc/blob/master/IMC.xml
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

# see mission_plan -> read_plandb
MANEUVER_GOTO = 450
MANEUVER_GOTO_STR = "goto"
MANEUVER_SAMPLE = 489
MANEUVER_SAMPLE_STR = "sample"
MANEUVER_COVER_AREA = 473
MANEUVER_COVER_AREA_STR = "cover_area"


# Speed units
SPEED_UNIT_MPS = 0
SPEED_UNIT_RPM = 1
SPEED_UNIT_PERCENTAGE = 2

# These are the same in smarc_msgs/GotoWaypoint.action
# Z units
Z_NONE = 0
Z_DEPTH = 1
Z_ALTITUDE = 2
Z_HEIGHT = 3



# a list of actions that we will consider
# as the imc 'executing' state
EXECUTING_ACTION_NAMES =[
    'A_GotoWaypoint',
    'A_FollowLeader',
    'A_SetNextPlanAction' #so we dont spam service/maneuver when going tru a lot of waypoints quickly
]
# same thing for the 'blocked' state
BLOCKED_ACTION_NAMES =[
    'A_EmergencySurface'
]


