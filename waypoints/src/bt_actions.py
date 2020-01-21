#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)
# Mostly a re-write of Christopher's behaviours with more
# descriptive names and atomicaztion of everything

#TODO:
# x A_SetMissionPlan
#  A_SetNextPlanAction
#  A_ExecutePlanAction
#  A_SetManualWaypoint
#  A_GotoManualWaypoint
#  A_PublishToNeptus

import Time

MISSION_PLAN_STR = 'plan_str'
MINIMUM_PLAN_STR_LEN = 135 # somehow Chris found this number, i'll reuse this
MISSION_PLAN_OBJ = 'misison_plan'

class MissionPlan:
    def __init__(self,
                 waypoints,
                 utm_zone,
                 utm_band):
        """
        A container object to keep things related to the mission plan.
        """
        #TODO enable other actions other than just waypoints

        self.waypoints = waypoints
        self.utm_zone = utm_zone
        self.utm_band = utm_band
        self.time_received = Time.now()

        self.remaining_wps = waypoints
        self.visited_wps = []
        self.current_wp = None

    def pop_wp(self):
        """
        Move a wp from remaining_wps to visit_wps
        """
        self.current_wp = self.remaining_wps[0]
        self.remaining_wps = self.remaining_wps[1:]
        return self.current_wp

    def visit(self):
        self.visited_wps.append(self.current_wp)
        self.current_wp = None




class A_SetMissionPlan(pt.behaviour.Behaviour):
    def __init__(self):
        """
        Reads the mission plan string from the black board
        and creates a new Mission object.

        returns SUCCESS if there was a plan and we successfully set it
        returns FAILURE otherwise
        """
        self.bb = pt.blackboard.Blackboard()
        super(A_SetMissionPlan, self).__init__('A_SetMissionPlan')

    def update(self):
        plan_str = self.bb.get(MISSION_PLAN_STR)

        # there was no plan to be set
        if plan_str is None or  len(plan_str) < MINIMUM_PLAN_STR_LEN:
            return pt.common.Status.FAILURE

        # there is a plan we can at least look at
        wps, zone, band = self.clean(plan_str)
        mission_plan = MissionPlan(waypoints=wps,
                                   utm_zone = zone,
                                   utm_band = band)

        self.bb.set(MISSION_PLAN_OBJ, mission_plan)
        return pt.common.Status.SUCCESS

    @staticmethod
    def clean(f):
        """
        Given a json printed string from imc_ros_bridge, cleans it up
        parses it and returns a list of utm xyz waypoints and the utm zone

        By: Christopher Iliffe Sprague (sprague@kth.se)
        """

        # make the neptus message into a string
        f = str(f)

        # clean the neptus message
        f = f.replace(' ', '')
        f = f.replace('\\n', '')
        f = f.replace('\\"', '"')
        f = f.replace('"\\', '"')
        f = f.replace('\\', '')
        f = f.split(',"transitions":')[0]
        f = f.split('"maneuvers":')[1]
        f = f.replace('\n', '')
        f = f.split(',"transitions"')[0]

        # convert to json
        f = json.loads(f)

        #json.dump(f, open('plan.json', 'w'))

        # convert lat lon to utm
        depths = [float(d['data']['z']) for d in f]

        # ensure signs of depths
        depths = [-d if d > 0 else d for d in depths]

        # get waypoint types
        wtypes = [str(d['data']['abbrev']) for d in f]

        # get latitute and longitude
        f = [fromLatLong(np.degrees(float(d['data']['lat'])), np.degrees(float(d['data']['lon']))) for d in f]

        # get the grid-zone
        gz, band = f[0].gridZone()

        # convert utm to point
        f = [d.toPoint() for d in f]

        # convert point to xyz
        f = [(d.x, d.y, depth, wt) for d, depth, wt in zip(f, depths, wtypes)]

        # return list of utm xyz waypoints and the utm zone
        return f, gz, band


class A_SetNextPlanAction(pt.behaviour.Behaviour):
    def __init__(self):
        """
        Sets the current plan action to the next one
        """
        self.bb = pt.blackboard.Blackboard()
        super(A_SetNextPlanAction, self).__init__('A_SetNextPlanAction')

    def update(self):

        self.bb.set(

