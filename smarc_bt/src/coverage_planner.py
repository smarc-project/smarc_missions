#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


import numpy as np
from numpy import *

def mirror(pts, axis=0):
    pts = np.array(pts)
    mid = np.mean(pts, axis=0)
    centered = pts - mid
    mirrored = centered[:,axis]*-1
    if axis == 0:
        ret = np.array(list(zip(mirrored+mid[0], pts[:,1])))
    if axis == 1:
        ret = np.array(list(zip(pts[:,0], mirrored+mid[1])))
    return ret


def minBoundingRect(hull_points_2d):
    # https://github.com/dbworth/minimum-area-bounding-rectangle/blob/master/python/min_bounding_rect.py
    # Find the minimum-area bounding box of a set of 2D points
    #
    # The input is a 2D convex hull, in an Nx2 numpy array of x-y co-ordinates.
    # The first and last points points must be the same, making a closed polygon.
    # This program finds the rotation angles of each edge of the convex polygon,
    # then tests the area of a bounding box aligned with the unique angles in
    # 90 degrees of the 1st Quadrant.
    # Returns the
    #
    # Tested with Python 2.6.5 on Ubuntu 10.04.4
    # Results verified using Matlab

    # Copyright (c) 2013, David Butterworth, University of Queensland
    # All rights reserved.
    #
    # Redistribution and use in source and binary forms, with or without
    # modification, are permitted provided that the following conditions are met:
    #
    #     * Redistributions of source code must retain the above copyright
    #       notice, this list of conditions and the following disclaimer.
    #     * Redistributions in binary form must reproduce the above copyright
    #       notice, this list of conditions and the following disclaimer in the
    #       documentation and/or other materials provided with the distribution.
    #     * Neither the name of the Willow Garage, Inc. nor the names of its
    #       contributors may be used to endorse or promote products derived from
    #       this software without specific prior written permission.
    #
    # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    # ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    # LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    # CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    # SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    # INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    # CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    # POSSIBILITY OF SUCH DAMAGE.
    #print "Input convex hull points: "
    #print hull_points_2d

    # Compute edges (x2-x1,y2-y1)
    edges = zeros( (len(hull_points_2d)-1,2) ) # empty 2 column array
    for i in range( len(edges) ):
        edge_x = hull_points_2d[i+1,0] - hull_points_2d[i,0]
        edge_y = hull_points_2d[i+1,1] - hull_points_2d[i,1]
        edges[i] = [edge_x,edge_y]
    #print "Edges: \n", edges

    # Calculate edge angles   atan2(y/x)
    edge_angles = zeros( (len(edges)) ) # empty 1 column array
    for i in range( len(edge_angles) ):
        edge_angles[i] = math.atan2( edges[i,1], edges[i,0] )
    #print "Edge angles: \n", edge_angles

    # Check for angles in 1st quadrant
    for i in range( len(edge_angles) ):
        edge_angles[i] = abs( edge_angles[i] % (math.pi/2) ) # want strictly positive answers
    #print "Edge angles in 1st Quadrant: \n", edge_angles

    # Remove duplicate angles
    edge_angles = unique(edge_angles)
    #print "Unique edge angles: \n", edge_angles

    # Test each angle to find bounding box with smallest area
    min_bbox = (0, np.iinfo(np.int32(10)).max, 0, 0, 0, 0, 0, 0) # rot_angle, area, width, height, min_x, max_x, min_y, max_y
    for i in range( len(edge_angles) ):

        # Create rotation matrix to shift points to baseline
        # R = [ cos(theta)      , cos(theta-PI/2)
        #       cos(theta+PI/2) , cos(theta)     ]
        R = array([ [ math.cos(edge_angles[i]), math.cos(edge_angles[i]-(math.pi/2)) ], [ math.cos(edge_angles[i]+(math.pi/2)), math.cos(edge_angles[i]) ] ])
        #print "Rotation matrix for ", edge_angles[i], " is \n", R

        # Apply this rotation to convex hull points
        rot_points = dot(R, transpose(hull_points_2d) ) # 2x2 * 2xn
        #print "Rotated hull points are \n", rot_points

        # Find min/max x,y points
        min_x = nanmin(rot_points[0], axis=0)
        max_x = nanmax(rot_points[0], axis=0)
        min_y = nanmin(rot_points[1], axis=0)
        max_y = nanmax(rot_points[1], axis=0)
        #print "Min x:", min_x, " Max x: ", max_x, "   Min y:", min_y, " Max y: ", max_y

        # Calculate height/width/area of this bounding rectangle
        width = max_x - min_x
        height = max_y - min_y
        area = width*height
        #print "Potential bounding box ", i, ":  width: ", width, " height: ", height, "  area: ", area

        # Store the smallest rect found first (a simple convex hull might have 2 answers with same area)
        if (area < min_bbox[1]):
            min_bbox = ( edge_angles[i], area, width, height, min_x, max_x, min_y, max_y )
        # Bypass, return the last found rect
        #min_bbox = ( edge_angles[i], area, width, height, min_x, max_x, min_y, max_y )

    # Re-create rotation matrix for smallest rect
    angle = min_bbox[0]
    R = array([ [ math.cos(angle), math.cos(angle-(math.pi/2)) ], [ math.cos(angle+(math.pi/2)), math.cos(angle) ] ])
    #print "Projection matrix: \n", R

    # Project convex hull points onto rotated frame
    proj_points = dot(R, transpose(hull_points_2d) ) # 2x2 * 2xn
    #print "Project hull points are \n", proj_points

    # min/max x,y points are against baseline
    min_x = min_bbox[4]
    max_x = min_bbox[5]
    min_y = min_bbox[6]
    max_y = min_bbox[7]
    #print "Min x:", min_x, " Max x: ", max_x, "   Min y:", min_y, " Max y: ", max_y

    # Calculate center point and project onto rotated frame
    center_x = (min_x + max_x)/2
    center_y = (min_y + max_y)/2
    center_point = dot( [ center_x, center_y ], R )
    #print "Bounding box center point: \n", center_point

    # Calculate corner points and project onto rotated frame
    corner_points = zeros( (4,2) ) # empty 2 column array
    corner_points[0] = dot( [ max_x, min_y ], R )
    corner_points[1] = dot( [ min_x, min_y ], R )
    corner_points[2] = dot( [ min_x, max_y ], R )
    corner_points[3] = dot( [ max_x, max_y ], R )
    #print "Bounding box corner points: \n", corner_points

    #print "Angle of rotation: ", angle, "rad  ", angle * (180/math.pi), "deg"

    return (angle, min_bbox[1], min_bbox[2], min_bbox[3], center_point, corner_points) # rot_angle, area, width, height, center_point, corner_points



def create_mower_pattern(rect_width_d, rect_height_h, sweep_width_w, error_growth_k):
    def s_next(s, k, b):
        return (1.0 + k) * s / (1.0 - k) + 2.0 * k * b / (1.0 - k)

    def length_wp_path(wp_list_x, wp_list_y):
        sum = 0.0;
        for i in range(len(wp_list_x) - 1):
            sum += math.sqrt(math.pow(wp_list_x[i] - wp_list_x[i + 1], 2)
                             + math.pow(wp_list_y[i] - wp_list_y[i + 1], 2))
        return sum

    pos_x = 0.0
    pos_y = sweep_width_w / 2.0
    wp_list_x = [pos_x] # start in the bottom left corner
    wp_list_y = [pos_y]
    b = sweep_width_w / (1.0 + error_growth_k)
    s_1 = (rect_width_d + b * error_growth_k) / (1.0 - error_growth_k)
    pos_x += s_1
    pos_y += -s_1 * error_growth_k
    wp_list_x.append(pos_x)
    wp_list_y.append(pos_y)
    s_old = s_1
    for i in range(23):
        pos_y += b
        wp_list_x.append(pos_x)
        wp_list_y.append(pos_y)
        s_new = s_next(s_old, error_growth_k, b)
        c_i = sweep_width_w - error_growth_k * (s_new + s_old + b)
        #pos_y += -b
        #pos_y += c_i
        pos_y = wp_list_y[-3] + c_i
        pos_x += -s_new
        wp_list_x.append(pos_x)
        wp_list_y.append(pos_y)
        if rect_height_h < pos_y + sweep_width_w / 2.0 - length_wp_path(wp_list_x, wp_list_y) * error_growth_k:
            break
        pos_y += b
        wp_list_x.append(pos_x)
        wp_list_y.append(pos_y)
        s_old = s_new
        s_new = s_next(s_old, error_growth_k, b)
        c_i = sweep_width_w - error_growth_k * (s_new + s_old + b)
        #pos_y += -b
        #pos_y += c_i
        pos_y = wp_list_y[-3] + c_i
        pos_x += s_new
        wp_list_x.append(pos_x)
        wp_list_y.append(pos_y)
        s_old = s_new
        if rect_height_h < pos_y + sweep_width_w / 2.0 - length_wp_path(wp_list_x, wp_list_y) * error_growth_k:
            break
    return (wp_list_x, wp_list_y)


def rotate_vec_vec(v1s, rads):
    """
    v1s is an array of shape (N,2) where each row is a vector
    rotates ALL of them rads radians around the origin
    """
    x1s = v1s[:,0]
    y1s = v1s[:,1]

    x2s = np.cos(rads)*x1s - np.sin(rads)*y1s
    y2s = np.sin(rads)*x1s + np.cos(rads)*y1s

    res = np.zeros_like(v1s)
    res[:,0] = x2s
    res[:,1] = y2s

    return res




def create_coverage_path(polygon, swath, error_growth):
    polygon = np.array(polygon)
    # the poly needs to be closed -> first and last elements need to be identical
    if not all(polygon[0] == polygon[-1]):
        polygon = np.vstack([polygon, polygon[0]])

    rot_angle, area, w, h, center, corners = minBoundingRect(polygon)

    # we want to do rows in the longest direction
    # so if the box is higher than its wider, turn it around
    flip = False
    if h>w:
        w,h = h,w
        flip = True

    # this is starting at 0,0 and going +y always
    coverage_xs, coverage_ys = create_mower_pattern(w, h, swath, error_growth)
    coverage_path = np.array(list(zip(coverage_xs, coverage_ys)))
    # first, center this path on 0,0
    coverage_path[:,0] -= np.mean(coverage_path[:,0])
    coverage_path[:,1] -= np.mean(coverage_path[:,1])
    # then rotate it to match the rotation of the bounding rect
    coverage_path = rotate_vec_vec(coverage_path, rot_angle)
    # rotate it a further 90 deg if we need to flip
    if flip:
        coverage_path = rotate_vec_vec(coverage_path, np.pi/2)

    # then translate to the bounding box locale
    coverage_path[:,0] += center[0]
    coverage_path[:,1] += center[1]

    # now we got the shape, we need to flip it around until
    # the starting point of the path is closest it can be to
    # the first vertex of the given polygon
    xm = mirror(coverage_path, axis=0)
    ym = mirror(coverage_path, axis=1)
    xym = mirror(xm, axis=1)

    versions = [coverage_path, xm, ym, xym]
    dists = [np.sum(np.abs(v[0]-polygon[0])) for v in versions]

    closest_version_idx = np.argmin(dists)
    closest_version = versions[closest_version_idx]


    return closest_version



if __name__ == '__main__':
    import matplotlib.pyplot as plt
    try:
        __IPYTHON__
        plt.ion()
    except:
        pass

    w = 50
    h = 20
    z = 20
    polygon = [[z ,z],
               [z, z+h],
               [z+w,z+h],
               [z+w,z]]
    polygon = np.array(polygon)
    polygon = rotate_vec_vec(polygon, np.pi)
    polygon = rotate_vec_vec(polygon, np.pi/4)

    plt.scatter(polygon[:,0], polygon[:,1])
    for i,p in enumerate(polygon):
        plt.text(p[0], p[1], s=str(i))

    swath = 5
    error_growth = 0.01

    coverage_path = create_coverage_path(polygon, swath, error_growth)
    plt.plot(coverage_path[:,0], coverage_path[:,1])

    for i,p in enumerate(coverage_path):
        plt.text(p[0], p[1], s=str(i))
    plt.axis('equal')








