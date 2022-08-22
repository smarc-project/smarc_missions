#!/usr/bin/python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

"""
Compute Dubins path between waypoints
"""

import math
import numpy as np
from enum import Enum

import rospy
import bb_enums
from tf.transformations import quaternion_from_euler
from smarc_msgs.msg import GotoWaypoint

class TurnType(Enum):
    LSL = 1
    LSR = 2
    RSL = 3
    RSR = 4
    RLR = 5
    LRL = 6

class Waypoint:

    def __init__(self, x, y, psi):
        self.x = x
        self.y = y
        self.psi = psi

    def __str__(self):
        return 'x: {x}, y: {y}, psi: {psi} + str(self.x)'.format(x=self.x, y=self.y, psi=self.psi)

class Param:
    def __init__(self, p_init, seg_final, turn_radius,):
        self.p_init = p_init
        self.seg_final = seg_final
        self.turn_radius = turn_radius
        self.type = 0

class Trajectory:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def wrapTo180(angle):
    angle =  angle % 360
    angle = (angle + 360) % 360
    if (angle > 180):
        angle -= 360;
    return angle

def calc_dubins_path(wpt1, wpt2, turn_radius):
    # Calculate a dubins path between two waypoints
    param = Param(wpt1, 0, 0)
    tz        = [0, 0, 0, 0, 0, 0]
    pz        = [0, 0, 0, 0, 0, 0]
    qz        = [0, 0, 0, 0, 0, 0]
    param.seg_final = [0, 0, 0]
    psi1 = wrapTo180(wpt1.psi)*math.pi/180
    psi2 = wrapTo180(wpt2.psi)*math.pi/180

    param.turn_radius = turn_radius
    dx = wpt2.x - wpt1.x
    dy = wpt2.y - wpt1.y
    D = math.sqrt(dx*dx + dy*dy)
    d = D/param.turn_radius # Normalize by turn radius

    theta = math.atan2(dy,dx) % (2*math.pi)
    alpha = (psi1 - theta) % (2*math.pi)
    beta  = (psi2 - theta) % (2*math.pi)
    best_word = -1
    lowest_cost = -1

    # Compute all Dubins paths between points
    tz[0], pz[0], qz[0] = dubinsLSL(alpha,beta,d)
    tz[1], pz[1], qz[1] = dubinsLSR(alpha,beta,d)
    tz[2], pz[2], qz[2] = dubinsRSL(alpha,beta,d)
    tz[3], pz[3], qz[3] = dubinsRSR(alpha,beta,d)
    tz[4], pz[4], qz[4] = dubinsRLR(alpha,beta,d)
    tz[5], pz[5], qz[5] = dubinsLRL(alpha,beta,d)

    # Pick the path with the lowest cost
    for k in range(len(tz)):
        if(tz[k]!=-1):
            cost = tz[k] + pz[k] + qz[k]
            if(cost<lowest_cost or lowest_cost==-1):
                best_word = k+1
                lowest_cost = cost
                param.seg_final = [tz[k],pz[k],qz[k]]

    param.type = TurnType(best_word)
    return param

# Compute all Dubins options
def dubinsLSL(alpha, beta, d):
    tmp0      = d + math.sin(alpha) - math.sin(beta)
    tmp1      = math.atan2((math.cos(beta)-math.cos(alpha)),tmp0)
    p_squared = 2 + d*d - (2*math.cos(alpha-beta)) + (2*d*(math.sin(alpha)-math.sin(beta)))
    if p_squared<0:
        # print('No LSL Path')
        p=-1
        q=-1
        t=-1
    else:
        t         = (tmp1-alpha) % (2*math.pi)
        p         = math.sqrt(p_squared)
        q         = (beta - tmp1) % (2*math.pi)
    return t, p, q

def dubinsRSR(alpha, beta, d):
    tmp0      = d - math.sin(alpha) + math.sin(beta)
    tmp1      = math.atan2((math.cos(alpha)-math.cos(beta)),tmp0)
    p_squared = 2 + d*d - (2*math.cos(alpha-beta)) + 2*d*(math.sin(beta)-math.sin(alpha))
    if p_squared<0:
        # print('No RSR Path')
        p=-1
        q=-1
        t=-1
    else:
        t         = (alpha - tmp1 ) % (2*math.pi)
        p         = math.sqrt(p_squared)
        q         = (-1*beta + tmp1) % (2*math.pi)
    return t, p, q

def dubinsRSL(alpha,beta,d):
    tmp0      = d - math.sin(alpha) - math.sin(beta)
    p_squared = -2 + d*d + 2*math.cos(alpha-beta) - 2*d*(math.sin(alpha) + math.sin(beta))
    if p_squared<0:
        # print('No RSL Path')
        p=-1
        q=-1
        t=-1
    else:
        p         = math.sqrt(p_squared)
        tmp2      = math.atan2((math.cos(alpha)+math.cos(beta)),tmp0) - math.atan2(2,p)
        t         = (alpha - tmp2) % (2*math.pi)
        q         = (beta - tmp2) % (2*math.pi)
    return t, p, q

def dubinsLSR(alpha, beta, d):
    tmp0      = d + math.sin(alpha) + math.sin(beta)
    p_squared = -2 + d*d + 2*math.cos(alpha-beta) + 2*d*(math.sin(alpha) + math.sin(beta))
    if p_squared<0:
        # print('No LSR Path')
        p=-1
        q=-1
        t=-1
    else:
        p         = math.sqrt(p_squared)
        tmp2      = math.atan2((-1*math.cos(alpha)-math.cos(beta)),tmp0) - math.atan2(-2,p)
        t         = (tmp2 - alpha) % (2*math.pi)
        q         = (tmp2 - beta) % (2*math.pi)
    return t, p, q

def dubinsRLR(alpha, beta, d):
    tmp_rlr = (6 - d*d + 2*math.cos(alpha-beta) + 2*d*(math.sin(alpha)-math.sin(beta)))/8
    if(abs(tmp_rlr)>1):
        # print('No RLR Path')
        p=-1
        q=-1
        t=-1
    else:
        p = (2*math.pi - math.acos(tmp_rlr)) % (2*math.pi)
        t = (alpha - math.atan2((math.cos(alpha)-math.cos(beta)), d-math.sin(alpha)+math.sin(beta)) + p/2 % (2*math.pi)) % (2*math.pi)
        q = (alpha - beta - t + (p % (2*math.pi))) % (2*math.pi)

    return t, p, q

def dubinsLRL(alpha, beta, d):
    tmp_lrl = (6 - d*d + 2*math.cos(alpha-beta) + 2*d*(-1*math.sin(alpha)+math.sin(beta)))/8
    if(abs(tmp_lrl)>1):
        # print('No LRL Path')
        p=-1
        q=-1
        t=-1
    else:
        p = (2*math.pi - math.acos(tmp_lrl)) % (2*math.pi)
        t = (-1*alpha - math.atan2((math.cos(alpha)-math.cos(beta)), d+math.sin(alpha)-math.sin(beta)) + p/2) % (2*math.pi)
        q = ((beta % (2*math.pi))-alpha-t+(p % (2*math.pi))) % (2*math.pi)
        # print(t,p,q,beta,alpha)
    return t, p, q

# Build the trajectory from the lowest-cost path
def dubins_traj(param,step):
    x = 0
    i = 0
    length = (param.seg_final[0]+param.seg_final[1]+param.seg_final[2])*param.turn_radius
    length = int(math.floor(length/step))
    path = -1 * np.ones((length,3))

    while x < length:
        path[i] = dubins_path(param,x)
        x += step
        i+=1
    return path

# Helper function for curve generation
def dubins_path(param, t):
    tprime = t/param.turn_radius
    p_init = np.array([0,0,wrapTo180(param.p_init.psi)*math.pi/180])
    #
    L_SEG = 1
    S_SEG = 2
    R_SEG = 3
    DIRDATA = np.array([[L_SEG,S_SEG,L_SEG],[L_SEG,S_SEG,R_SEG],[R_SEG,S_SEG,L_SEG],[R_SEG,S_SEG,R_SEG],[R_SEG,L_SEG,R_SEG],[L_SEG,R_SEG,L_SEG]])
    #
    types = DIRDATA[param.type.value-1][:]
    param1 = param.seg_final[0]
    param2 = param.seg_final[1]
    mid_pt1 = dubins_segment(param1,p_init,types[0])
    mid_pt2 = dubins_segment(param2,mid_pt1,types[1])

    if(tprime<param1):
        end_pt = dubins_segment(tprime,p_init,types[0])
    elif(tprime<(param1+param2)):
        end_pt = dubins_segment(tprime-param1,mid_pt1,types[1])
    else:
        end_pt = dubins_segment(tprime-param1-param2, mid_pt2, types[2])

    end_pt[0] = end_pt[0] * param.turn_radius + param.p_init.x
    end_pt[1] = end_pt[1] * param.turn_radius + param.p_init.y
    end_pt[2] = wrapTo180(math.degrees(end_pt[2] % (2*math.pi))) # [deg]

    return end_pt

def dubins_segment(seg_param, seg_init, seg_type):
    L_SEG = 1
    S_SEG = 2
    R_SEG = 3
    seg_end = np.array([0.0,0.0,0.0])
    if( seg_type == L_SEG ):
        seg_end[0] = seg_init[0] + math.sin(seg_init[2]+seg_param) - math.sin(seg_init[2])
        seg_end[1] = seg_init[1] - math.cos(seg_init[2]+seg_param) + math.cos(seg_init[2])
        seg_end[2] = seg_init[2] + seg_param
    elif( seg_type == R_SEG ):
        seg_end[0] = seg_init[0] - math.sin(seg_init[2]-seg_param) + math.sin(seg_init[2])
        seg_end[1] = seg_init[1] + math.cos(seg_init[2]-seg_param) - math.cos(seg_init[2])
        seg_end[2] = seg_init[2] - seg_param
    elif( seg_type == S_SEG ):
        seg_end[0] = seg_init[0] + math.cos(seg_init[2]) * seg_param
        seg_end[1] = seg_init[1] + math.sin(seg_init[2]) * seg_param
        seg_end[2] = seg_init[2]

    return seg_end

# Compute angles between waypoints and return list of Waypoints objects
# Waypoints in input only have [x,y] coordinates
def waypoints_with_yaw(waypoints):
    pts = waypoints
    angles = []
    Wptz = []
    for i in (range(len(pts))):
        if i != len(pts)-1:
            angle = math.degrees(math.atan2(pts[i+1,1] - pts[i,1], pts[i+1,0] - pts[i,0]))
        else:
            angle = math.degrees(math.atan2(pts[len(pts)-1,1] - pts[len(pts)-2,1], pts[len(pts)-1,0] - pts[len(pts)-2,0]))
        angles.append(angle)
        Wptz.append(Waypoint(pts[i,0], pts[i,1], angle))
    return Wptz, angles

def circle_line_segment_intersection(circle_center, circle_radius, pt1, pt2, full_line=False, tangent_tol=1e-5):
    """ Find the points at which a circle intersects a line-segment.  This can happen at 0, 1, or 2 points.

    :param circle_center: The (x, y) location of the circle center
    :param circle_radius: The radius of the circle
    :param pt1: The (x, y) location of the first point of the segment
    :param pt2: The (x, y) location of the second point of the segment
    :param full_line: True to find intersections along full line - not just in the segment.  False will just return intersections within the segment.
    :param tangent_tol: Numerical tolerance at which we decide the intersections are close enough to consider it a tangent
    :return Sequence[Tuple[float, float]]: A list of length 0, 1, or 2, where each element is a point at which the circle intercepts a line segment.

    Note: We follow: http://mathworld.wolfram.com/Circle-LineIntersection.html
    """

    (p1x, p1y), (p2x, p2y), (cx, cy) = pt1, pt2, circle_center
    (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
    dx, dy = (x2 - x1), (y2 - y1)
    dr = (dx ** 2.0 + dy ** 2.0)**.5
    big_d = x1 * y2 - x2 * y1
    discriminant = circle_radius ** 2.0 * dr ** 2.0 - big_d ** 2.0

    if discriminant < 0:  # No intersection between circle and line
        return []
    else:  # There may be 0, 1, or 2 intersections with the segment
        intersections = [
            (cx + (big_d * dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**.5) / dr ** 2.0,
                cy + (-big_d * dx + sign * abs(dy) * discriminant**.5) / dr ** 2.0)
            for sign in ((1, -1) if dy < 0 else (-1, 1))]  # This makes sure the order along the segment is correct
        if not full_line:  # If only considering the segment, filter out intersections that do not fall within the segment
            fraction_along_segment = [
                (xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy for xi, yi in intersections]
            intersections = [pt for pt, frac in zip(
                intersections, fraction_along_segment) if 0 <= frac <= 1]
        # If line is tangent to circle, return just one point (as both intersections have same location)
        if len(intersections) == 2 and abs(discriminant) <= tangent_tol:
            return [intersections[0]]
        else:
            return intersections


def dubins_mission_planner(mission, bb, ll_to_utm_serv, utm_to_ll_serv, utm_to_latlon, latlon_to_utm, num_points=2, inside_turn=True):
    '''
    Reads the waypoints from a MissionControl message and generates a sampled dubins
    path between them.

    :param mission: the original MissionControl message
    :param num_points: the amount of waypoints to keep on each segment between waypoints
    :param inside_turn: cut the waypoints corners
    :return MissionControl.msg: new MissionControl message equal to the input one exc   ept for the waypoints attribute
    '''

    rospy.loginfo("Computing dubins path")

    turn_radius = bb.get(bb_enums.DUBINS_TURNING_RADIUS)
    int_radius = bb.get(bb_enums.DUBINS_INTERSECTION_RADIUS)
    vehicle = bb.get(bb_enums.VEHICLE_STATE)
    robot_name = bb.get(bb_enums.ROBOT_NAME)

    # Get the auv's current position to use as the intial point of the dubins path
    utm_x_auv, utm_y_auv = vehicle.position_utm
    auv_utm = np.array([utm_x_auv, utm_y_auv])

    # Get x, y of waypoints from original mission lat/lon
    points = []
    for wp in mission.waypoints:
        utm_x, utm_y = latlon_to_utm(
            wp.lat, wp.lon, wp.pose.pose.position.z, serv=ll_to_utm_serv, in_degrees=True)
        points.append([utm_x, utm_y])
    points_np = np.array(points)
    points_with_auv = np.vstack((auv_utm, points_np))

    if not inside_turn:
        rospy.loginfo("Turning outside")
        original_waypoints = points_with_auv
    else:
        rospy.loginfo("Turning inside")
        waypoints = []
        for i in range(len(points_with_auv)-1):
            waypoints_i = circle_line_segment_intersection(circle_center=(points_with_auv[i, 0], points_with_auv[i, 1]), circle_radius=int_radius,
                                                            pt1=(points_with_auv[i, 0], points_with_auv[i, 1]), pt2=(points_with_auv[i+1, 0], points_with_auv[i+1, 1]))
            waypoints.append(waypoints_i)
            waypoints_j = circle_line_segment_intersection(circle_center=(points_with_auv[i+1, 0], points_with_auv[i+1, 1]), circle_radius=int_radius,
                                                            pt1=(points_with_auv[i, 0], points_with_auv[i, 1]), pt2=(points_with_auv[i+1, 0], points_with_auv[i+1, 1]))
            waypoints.append(waypoints_j)

        waypoints.append(
            [(points_with_auv[-1, 0], points_with_auv[-1, 1])])
        # Flatten list of lists
        waypoints = [el for sublist in waypoints for el in sublist]
        waypoints_np = np.array(waypoints)
        original_waypoints = waypoints_np.squeeze()  # Remove useless extra dimension

    # Keep the other waypoints' parameters equal
    mwp = mission.waypoints[0]
    goal_tolerance = mwp.goal_tolerance
    z_control_mode = mwp.z_control_mode
    travel_altitude = mwp.travel_altitude
    travel_depth = mwp.travel_depth
    speed_control_mode = mwp.speed_control_mode
    travel_rpm = mwp.travel_rpm
    travel_speed = mwp.travel_speed

    # Compute angle between waypoints and get the new wps array
    waypoints_complete, _ = waypoints_with_yaw(original_waypoints)

    path = []
    for j in range(len(waypoints_complete)-1):
        param = calc_dubins_path(
            waypoints_complete[j], waypoints_complete[j+1], turn_radius)
        path.append(dubins_traj(param, 1))

    dubins_waypoints = []
    # Only define the new waypoints on the original waypoint and on the curve
    # Each element of path is an array of points between on waypoint and the next one
    for i, el in enumerate(path):
        # Compute the differnece between orientation
        # The maxima represent the points on the curve
        delta_angles = abs(np.diff(el[:, 2]))
        delta_angles = [0] + delta_angles
        # Keep two points in each curve
        max_idxs = np.argpartition(delta_angles, -num_points)[-num_points:]
        max_idxs = np.sort(max_idxs)
        for ind in max_idxs:
            # A row of path[i] represents a single point
            dubins_waypoints.append(el[ind])

    # Sort the waypoints
    full_path = []
    ordered_idxs = []
    for el in path:
        for e in el:
            full_path.append([e[0], e[1], e[2]])
    for j, el in enumerate(dubins_waypoints):
        try:
            # Get the dubins wp index in the full path
            ordered_idxs.append(full_path.index([el[0], el[1], el[2]]))
        except ValueError:
            rospy.logwarn("Point " + str(j) + " not found in the path!")
    ordered_idxs = np.sort(ordered_idxs)
    dubins_waypoints_ordered = [full_path[i] for i in ordered_idxs]

    # Include original waypoints
    # The first one is the auv's position, we don't need it
    wp_i = num_points
    for wp in waypoints_complete[1:-1]:
        dubins_waypoints_ordered.insert(wp_i, [wp.x, wp.y, wp.psi])
        wp_i += num_points + 1
    dubins_waypoints_ordered.append(
        [waypoints_complete[-1].x, waypoints_complete[-1].y, waypoints_complete[-1].psi])
    dubins_waypoints_final = dubins_waypoints_ordered[1:]

    dubins_mission = mission
    del dubins_mission.waypoints[:]

    current_wp = -1
    dwp_count = 1
    for wp in dubins_waypoints_final:
        dwp = GotoWaypoint()
        dwp.pose.pose.position.x = wp[0]
        dwp.pose.pose.position.y = wp[1]
        dwp.pose.pose.position.z = travel_altitude
        dwp.lat, dwp.lon = utm_to_latlon(wp[0], wp[1], utm_to_ll_serv)

        quaternion = quaternion_from_euler(
            0.0, 0.0, math.radians(wp[2]))  # RPY [rad]
        # Normalize quaternion
        if len(quaternion) != 0:
            quaternion = quaternion / np.sqrt(np.sum(quaternion**2))
        dwp.pose.pose.orientation.x = quaternion[0]
        dwp.pose.pose.orientation.y = quaternion[1]
        dwp.pose.pose.orientation.z = quaternion[2]
        dwp.pose.pose.orientation.w = quaternion[3]

        dwp.goal_tolerance = goal_tolerance
        dwp.z_control_mode = z_control_mode
        dwp.travel_altitude = travel_altitude
        dwp.travel_depth = travel_depth
        dwp.speed_control_mode = speed_control_mode

        dwp.travel_speed = travel_speed
        dwp.travel_rpm = travel_rpm

        # Name of the dwp = previousoriginalwp_nextoriginalwp_counter
        if current_wp == -1:
            dwp.name = robot_name + "_wp0_" + str(dwp_count)
        else:
            dwp.name = "wp_" + str(current_wp) + "_wp_" + \
                str(current_wp+1) + "_" + str(dwp_count)

        if dwp_count == num_points:
            dwp_count = 0
            current_wp += 1
        else:
            dwp_count += 1

        dubins_mission.waypoints.append(dwp)

    rospy.loginfo("Dubins mission ready")
    return dubins_mission
