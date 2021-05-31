#! /usr/bin/env python
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
                 save_location = "~/MissionLogs/"):
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
        save_folder = os.path.expanduser(save_location)
        self.disabled = False
        try:
            if not os.path.exists(save_folder):
                os.makedirs(save_folder)
            else:
                print("Log folder({}) exists".format(save_folder))
        except:
            print("Log folder({}) could not be created!".format(save_folder))
            self.disabled = True
            return

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
        if self.disabled:
            print("Save location was bad before, can not save!")
            return

        # save a json file for now for easy inspection
        data = {'navigation_trace':self.navigation_trace,
                'raw_gps_trace':self.raw_gps_trace,
                'tree_tip_trace':self.tree_tip_trace,
                'mission_plan_wps':self.mission_plan_wps}

        with open(self.save_location, 'w+') as f:
            json.dump(data, f)

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


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

    equal_z = False
    try:
        if sys.argv[2] == 'equal':
            equal_z = True
    except:
        pass




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
    ax = fig.add_subplot(111, projection='3d')


    plt.plot(nav_trace[:,0], nav_trace[:,1], nav_trace[:,2])
    plt.plot(mplan[:,0], mplan[:,1], mplan[:,2])

    # filter out "non-fixes"
    gps_fixes = gps_trace[gps_trace !=  None]
    if len(gps_fixes) > 0:
        gps_fixes -= origin
        plt.plot(gps_fixes[:,0], gps_fixes[:,1], gps_fixes[:,1])

    if equal_z:
        set_axes_equal(ax)


    plt.show()




