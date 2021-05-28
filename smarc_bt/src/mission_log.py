#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


import imc_enums
import numpy as np
import os
import json
import time

class MissionLog:
    def __init__(self,
                 mission_plan,
                 save_location = "/MissionLogs/"):
        """
        A log object to create easy to read and visualize path logs
        for individual missions.

        One log per mission_plan.

        Everything in utm frame and z up.
        """

        # filtered/corrected trace of auv pose
        # x,y,z, yaw,pitch,roll
        self.navigation_trace = []
        # raw gps fixes
        self.raw_gps_trace = []
        # leaf nodes with name and status
        self.tree_tip_trace = []
        # the waypoints of the mission this log belongs to
        self.mission_plan_wps = []

        # give a nice name to the file
        t = time.localtime(mission_plan.creation_time)
        t_str = "{}-{:02d}-{:02d}-{:02d}-{:02d}".format(
            t.tm_year,
            t.tm_mon,
            t.tm_mday,
            t.tm_hour,
            t.tm_min)

        log_filename = "{}_{}.json".format(t_str, mission_plan.plan_id)
        save_folder = os.path.join(os.path.expanduser("~"), save_location)
        os.makedirs(save_folder, exist_ok=True)
        self.save_location = os.path.join(save_folder, log_filename)

        # used to check if log and mission plan are synched
        self.creation_time = mission_plan.creation_time


        # we can populate the mission plan right away
        for wp in mission_plan.waypoints:
            if wp.z_unit == imc_enums.Z_DEPTH:
                z = -wp.z
            else:
                z = wp.z

            point = (wp.x, wp.y, z)
            self.mission_plan_wps.append(point)



    def save(self):
        # save a json file for now for easy inspection
        data = {'navigation_trace':self.navigation_trace,
                'raw_gps_trace':self.raw_gps_trace,
                'tree_tip_trace':self.tree_tip_trace,
                'mission_plan_wps':self.mission_plan_wps}

        with open(self.save_location, 'w+') as f:
            json.dump(data, f)




if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D #needed for '3d'
    import sys

    try:
        __IPYTHON__
        plt.ion()
    except:
        pass

    filename = sys.argv[1]
    filename = os.path.join(os.path.expanduser("~"), "MissionLogs", filename)

    with open(filename, 'r') as f:
        data = json.load(f)

    nav_trace = np.array(data['navigation_trace'])
    gps_trace = np.array(data['raw_gps_trace'])
    mplan = np.array(data['mission_plan_wps'])

    origin = np.array(list(nav_trace[0]))
    #center on first nav point
    nav_trace -= origin
    mplan -= origin

    fig = plt.figure()
    plt.axis('equal')
    ax = fig.add_subplot(111, projection='3d')


    plt.plot(nav_trace[:,0], nav_trace[:,1], nav_trace[:,2])
    plt.plot(mplan[:,0], mplan[:,1], mplan[:,2])

    # filter out "non-fixes"
    gps_fixes = gps_trace[gps_trace !=  None]
    if len(gps_fixes) > 0:
        gps_fixes -= origin
        plt.plot(gps_fixes[:,0], gps_fixes[:,1], gps_fixes[:,1])

    plt.show()




