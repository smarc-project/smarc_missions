#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


import imc_enums
import bb_enums
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
        # distance from bottom
        self.altitude_trace = []
        # raw gps fixes
        self.raw_gps_trace = []
        # leaf nodes with name and status
        self.tree_tip_trace = []
        # the waypoints of the mission this log belongs to
        self.mission_plan_wps = []

        self.time_trace = []


        if mission_plan is not None:
            # used to check if log and mission plan are synched
            self.creation_time = mission_plan.creation_time
            self.plan_id = mission_plan.plan_id

            # we can populate the mission plan right away
            for wp in mission_plan.waypoints:
                if wp.z_unit == imc_enums.Z_DEPTH:
                    z = -wp.z
                else:
                    z = wp.z

                point = (wp.x, wp.y, z)
                self.mission_plan_wps.append(point)
        else:
            self.creation_time = time.time()
            self.plan_id = "MANUAL"


        # give a nice name to the file
        t = time.localtime(self.creation_time)
        t_str = "{}-{:02d}-{:02d}-{:02d}-{:02d}".format(
            t.tm_year,
            t.tm_mon,
            t.tm_mday,
            t.tm_hour,
            t.tm_min)

        log_filename = "{}_{}.json".format(t_str, self.plan_id)
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



    def log(self, bb, mplan, t=None):
        # first add the auv pose
        world_trans = bb.get(bb_enums.WORLD_TRANS)
        x,y = world_trans[0], world_trans[1]
        z = -bb.get(bb_enums.DEPTH)
        roll = bb.get(bb_enums.ROLL)
        pitch = bb.get(bb_enums.PITCH)
        yaw = bb.get(bb_enums.YAW)
        self.navigation_trace.append((x,y,z, roll,pitch,yaw))

        # then add the raw gps
        gps = bb.get(bb_enums.RAW_GPS)
        if gps is None or gps.status.status == -1: # no fix
            gps_utm_point = None
        else:
            # translate the latlon to utm point using the same service as the mission plan
            gps_utm_x, gps_utm_y = mplan.latlon_to_utm(gps.latitude, gps.lonitude)
            if gps_utm_x is None or gps_utm_y is None:
                gps_utm_point = None
        self.raw_gps_trace.append(gps_utm_point)

        # then add the tree tip and its status
        tree_tip = bb.get(bb_enums.TREE_TIP_NAME)
        tip_status = bb.get(bb_enums.TREE_TIP_STATUS)
        self.tree_tip_trace.append((tree_tip, tip_status))

        # time keeping
        if t is None:
            t = time.time()
        self.time_trace.append(t)

        # simple enough
        alt = bb.get(bb_enums.ALTITUDE)
        self.altitude_trace.append(alt)



    def save(self):
        if self.disabled:
            print("Save location was bad before, can not save!")
            return

        # save a json file for now for easy inspection
        data = {'navigation_trace':self.navigation_trace,
                'raw_gps_trace':self.raw_gps_trace,
                'tree_tip_trace':self.tree_tip_trace,
                'mission_plan_wps':self.mission_plan_wps,
                'time_trace':self.time_trace,
                'altitude_trace':self.altitude_trace}

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
    loc_trace = nav_trace[:,:3]
    roll_trace = nav_trace[:,3]
    pitch_trace = nav_trace[:,4]
    yaw_trace = nav_trace[:,5]
    gps_trace = np.array(data['raw_gps_trace'])
    mplan = np.array(data['mission_plan_wps'])
    altitude_trace = np.array(data['altitude_trace'])


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    origin = np.array(list(loc_trace[0]))
    #center on first nav point
    loc_trace -= origin
    ax.plot(loc_trace[:,0], loc_trace[:,1], loc_trace[:,2], c='red')

    try:
        rot_vecs_x = np.cos(yaw_trace) * np.cos(pitch_trace)
        rot_vecs_y = np.sin(yaw_trace) * np.cos(pitch_trace)
        rot_vecs_z = np.sin(pitch_trace)
        rot_vecs = np.vstack([rot_vecs_x, rot_vecs_y, rot_vecs_z]).T
        step = 9
        ax.quiver(loc_trace[::step, 0],
                  loc_trace[::step, 1],
                  loc_trace[::step, 2],
                  rot_vecs[::step, 0],
                  rot_vecs[::step, 1],
                  rot_vecs[::step, 2],
                  length = 1)
    except:
        pass


    # plot the altitude relative to the height of the auv
    bottom = loc_trace[:,2] - altitude_trace
    plt.plot(loc_trace[:,0], loc_trace[:,1], bottom, c='brown')


    if len(mplan) > 1:
        mplan -= origin
        plt.plot(mplan[:,0], mplan[:,1], mplan[:,2], c='green')

    # filter out "non-fixes"
    gps_fixes = gps_trace[gps_trace !=  None]
    if len(gps_fixes) > 0:
        gps_fixes -= origin
        # plt.plot(gps_fixes[:,0], gps_fixes[:,1], gps_fixes[:,1])

    if equal_z:
        set_axes_equal(ax)


    plt.show()




