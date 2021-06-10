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

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
import rospy

class MissionLog:
    def __init__(self,
                 mission_plan,
                 robot_name,
                 save_location = "~/MissionLogs/"):
        """
        A log object to create easy to read and visualize path logs
        for individual missions.

        One log per mission_plan.

        Everything in utm frame and z up.
        """

        self.robot_name = robot_name
        # filtered/corrected trace of auv pose
        # x,y,z, yaw,pitch,roll
        self.navigation_trace = []
        # vx,vy,vz from dvl
        self.velocity_trace = []
        # distance from bottom
        self.altitude_trace = []
        # raw gps fixes
        self.raw_gps_trace = []
        self.raw_gps_latlon_trace = []
        # leaf nodes with name and status
        self.tree_tip_trace = []
        # the waypoints of the mission this log belongs to
        self.mission_plan_wps = []

        self.time_trace = []

        # a dict for vehicle-specific data
        # loaded from Log<Vehicle> actions
        self.vehicle_data = {}


        self.path_msg = Path()
        self.path_msg.header.frame_id = 'utm'
        self.path_pub = rospy.Publisher('bt_viz/path', Path, queue_size=1)

        self.bottom_msg = Path()
        self.bottom_msg.header.frame_id = 'utm'
        self.bottom_pub = rospy.Publisher('bt_viz/bottom', Path, queue_size=1)

        self.plan_pub = rospy.Publisher('bt_viz/mission_plan', Path, queue_size=1)
        self.target_pub = rospy.Publisher('bt_viz/target_wp', Marker, queue_size=1)

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

        self.data_full_path = os.path.join(save_folder, log_filename)
        script_name = 'view.py'
        self.script_full_path = os.path.join(save_folder, script_name)

        self.swath = None
        self.loc_uncertainty_growth = None


    def vehicle_log(self, key, bb_key, bb):
        l = self.vehicle_data.get(key)
        if l is None:
            self.vehicle_data[key] = []
        self.vehicle_data[key].append(bb.get(bb_key))


    def log_lolo(self, bb):
        if 'lolo' not in self.robot_name:
            return False

        self.vehicle_data['robot_name'] = self.robot_name
        self.vehicle_log('elevator_trace', bb_enums.LOLO_ELEVATOR, bb)
        self.vehicle_log('elevon_port_trace', bb_enums.LOLO_ELEVON_PORT, bb)
        self.vehicle_log('elevon_strb_trace', bb_enums.LOLO_ELEVON_STRB, bb)
        self.vehicle_log('aft_tank_trace', bb_enums.LOLO_AFT_TANK, bb)
        self.vehicle_log('aft_tank_target_trace', bb_enums.LOLO_AFT_TANK_TARGET, bb)
        self.vehicle_log('front_tank_trace', bb_enums.LOLO_FRONT_TANK, bb)
        self.vehicle_log('front_tank_target_trace', bb_enums.LOLO_FRONT_TANK_TARGET, bb)

        return True


    def log_sam(self, bb):
        if 'sam' not in self.robot_name:
            return False
        return True


    def log(self, bb, mplan, t=None):
        ############################################
        # vehicle-specific stuff
        logged_lolo = self.log_lolo(bb)
        logged_sam = self.log_sam(bb)


        ############################################
        # vehicle-agnostic stuff
        self.swath = bb.get(bb_enums.SWATH)
        self.loc_uncertainty_growth = bb.get(bb_enums.LOCALIZATION_ERROR_GROWTH)

        # first add the auv pose
        world_trans = bb.get(bb_enums.WORLD_TRANS)
        x,y = world_trans[0], world_trans[1]
        z = -bb.get(bb_enums.DEPTH)
        roll = bb.get(bb_enums.ROLL)
        pitch = bb.get(bb_enums.PITCH)
        yaw = bb.get(bb_enums.YAW)
        self.navigation_trace.append((x,y,z, roll,pitch,yaw))

        point = bb.get(bb_enums.LOCATION_POINT_STAMPED)
        ps = PoseStamped()
        ps.header = point.header
        ps.pose.position.x = point.point.x
        ps.pose.position.y = point.point.y
        ps.pose.position.z = point.point.z
        self.path_msg.poses.append(ps)
        self.path_pub.publish(self.path_msg)

        # velocities from dvl
        vel_msg = bb.get(bb_enums.DVL_VELOCITY)
        vels = (vel_msg.x, vel_msg.y, vel_msg.z)
        self.velocity_trace.append(vels)


        # then add the raw gps
        # but only if it is diffeent than the previous one?
        gps = bb.get(bb_enums.RAW_GPS)
        if gps is None or gps.status.status == -1 or abs(time.time() - gps.header.stamp.secs) > 10: # no fix
            self.raw_gps_latlon_trace.append(None)
            gps_utm_point = None
        else:
            # also log the raw lat lon
            self.raw_gps_latlon_trace.append((gps.latitude, gps.longitude))
            # translate the latlon to utm point using the same service as the mission plan
            gps_utm_x, gps_utm_y = mplan.latlon_to_utm(lat = gps.latitude,
                                                       lon = gps.longitude,
                                                       z = 0.,
                                                       in_degrees = True)
            if gps_utm_x is None or gps_utm_y is None:
                gps_utm_point = None
            else:
                gps_utm_point = (gps_utm_x, gps_utm_y)
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

        ps = PoseStamped()
        ps.header = point.header
        ps.pose.position.x = point.point.x
        ps.pose.position.y = point.point.y
        ps.pose.position.z = point.point.z - alt
        self.bottom_msg.poses.append(ps)
        self.bottom_pub.publish(self.bottom_msg)

        if self.plan_id != 'MANUAL':
            self.plan_msg = Path()
            self.plan_msg.header.frame_id = 'utm'
            for x,y,z in self.mission_plan_wps:
                ps = PoseStamped()
                ps.header = point.header
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.position.z = min(z,0)
                self.plan_msg.poses.append(ps)
            self.plan_pub.publish(self.plan_msg)

        current_loc = bb.get(bb_enums.WORLD_TRANS)
        mplan = bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mplan is not None and current_loc is not None:
            wp = mplan.get_current_wp()
            x,y,z = current_loc

            arrow = Marker()
            arrow.header.frame_id = 'utm'
            arrow.ns = 'bt_viz'
            arrow.id = 0
            arrow.type = 0 # arrow
            arrow.action = 0 # add/modify
            arrow.color.r = 1
            arrow.color.g = 0.2
            arrow.color.b = 0.2
            arrow.color.a = 0.5
            arrow.scale.x = 0.5
            arrow.scale.y = 0.7
            arrow.scale.z = 1
            p1 = Point()
            p1.x = x
            p1.y = y
            p1.z = z
            p2 = Point()
            p2.x = wp.x
            p2.y = wp.y
            p2.z = wp.z
            arrow.points = [p1,p2]
            self.target_pub.publish(arrow)






    def save(self):
        if self.disabled:
            print("Save location was bad before, can not save!")
            return

        # save a json file for now for easy inspection
        data = {'navigation_trace':self.navigation_trace,
                'velocity_trace':self.velocity_trace,
                'raw_gps_trace':self.raw_gps_trace,
                'raw_gps_latlon_trace':self.raw_gps_latlon_trace,
                'tree_tip_trace':self.tree_tip_trace,
                'mission_plan_wps':self.mission_plan_wps,
                'time_trace':self.time_trace,
                'altitude_trace':self.altitude_trace,
                'vehicle_data':self.vehicle_data,
                'swath':self.swath,
                'loc_uncertainty_growth':self.loc_uncertainty_growth}

        with open(self.data_full_path, 'w+') as f:
            json.dump(data, f)

        # also save a viewer script to the same locale
        try:
            with open(self.script_full_path, 'w+') as f:
                global viewer_script
                f.write(viewer_script)
        except:
            print("Viewer script could not be written")


viewer_script = """#! /usr/bin/env python3

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
    import numpy as np
    import os
    import json
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
    if 'equalz' in sys.argv:
        equal_z = True

    no_bottom = False
    if 'nobottom' in sys.argv:
        no_bottom = True

    no_swath = False
    if 'noswath' in sys.argv:
        no_swath = True




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
    swath = data.get('swath', 20)
    err_growth = data.get('loc_uncertainty_growth',0.1)


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    origin = np.array(list(loc_trace[0]))
    #center on first nav point
    loc_trace -= origin
    ax.plot(loc_trace[:,0], loc_trace[:,1], loc_trace[:,2], c='green')
    ax.text(loc_trace[0,0], loc_trace[0,1], loc_trace[0,2], "S")
    ax.text(loc_trace[-1,0], loc_trace[-1,1], loc_trace[-1,2], "E")

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
              length = 1,
              color='green')


    if not no_bottom:
        # plot the altitude relative to the height of the auv
        bottom = loc_trace[:,2] - altitude_trace
        ax.plot(loc_trace[:,0], loc_trace[:,1], bottom, c='yellow')

    if not no_swath and swath is not None:
        bottom = loc_trace[:,2] - altitude_trace
        bottom_mid = np.median(bottom)
        left_xs, left_ys = np.cos(yaw_trace+np.pi/2), np.sin(yaw_trace+np.pi/2)
        right_xs, right_ys = np.cos(yaw_trace-np.pi/2), np.sin(yaw_trace-np.pi/2)
        left_xs *= swath/2
        right_xs *= swath/2
        left_ys *= swath/2
        right_ys *= swath/2
        left_xs += loc_trace[:,0]
        right_xs += loc_trace[:,0]
        left_ys += loc_trace[:,1]
        right_ys += loc_trace[:,1]
        for x1,x2,y1,y2 in zip(right_xs, left_xs, right_ys, left_ys):
            ax.plot([x1,x2],[y1,y2],[bottom_mid, bottom_mid],
                    c='purple', alpha=0.2)





    if len(mplan) > 1:
        mplan[:,:2] -= origin[:2]
        mplan[:,2] = np.minimum(mplan[:,2], 0)
        ax.plot(mplan[:,0], mplan[:,1], mplan[:,2], c='red')

    # filter out "non-fixes"
    good_fixes = []
    good_fix_locs = []
    for gps_fix, loc in zip(gps_trace, loc_trace):
        if gps_fix is None:
            continue
        good_fixes.append(gps_fix-origin[:2])
        good_fix_locs.append(loc)

    if len(good_fixes) > 0:
        gps_fixes = np.array(good_fixes)
        fix_locs = np.array(good_fix_locs)

        ax.scatter(gps_fixes[:,0], gps_fixes[:,1], 0, c='grey', alpha=0.2)
        step = 20
        for fix, loc in zip(gps_fixes[::step], fix_locs[::step]):
            xs = [fix[0], loc[0]]
            ys = [fix[1], loc[1]]
            zs = [0., loc[2]]
            diff = np.sqrt((xs[0]-xs[1])**2+(ys[0]-ys[1])**2)
            if diff > 0.5:
                ax.plot(xs, ys, zs, c='grey', alpha=0.1)
                ax.text(sum(xs)/2, sum(ys)/2, sum(zs)/2, s='{:.1f}'.format(diff))

    plt.xlabel('lat utm')
    plt.ylabel('lon utm')

    tstart = data['time_trace'][0]
    tend = data['time_trace'][-1]
    plt.title("Duration:{:.2f} mins".format((tend-tstart)/60))

    if equal_z:
        set_axes_equal(ax)

    plt.show()
"""



