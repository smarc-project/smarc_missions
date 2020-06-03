#!/usr/bin/env python3

# Christopher Iliffe Sprague
# Parent class, describing a ROS service that takes
# a PoseArray of 'coarse waypoints' and returns a Path.
# Usage: 'imc(utm) -> bt -> poseArray(map) -> path_planner -> path(map) -> bt'

import rospy, numpy as np, matplotlib.pyplot as plt
from scipy.interpolate import interp1d

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from nav_msgs.msg import Path
from trajectories.srv import trajectory

class Trajectory:

    def __init__(self, name):

        # start node
        self.name = name
        rospy.init_node(self.name)
        rospy.Service(self.name, trajectory, self.__call__)

    def __call__(self, trajectory_request):

        # sanity
        pose_array = trajectory_request.coarse
        assert isinstance(pose_array, PoseArray), \
            "Argument not PoseArray, it's a {}".format(type(pose_array))

        # convert PoseArray to numpy
        y = np.array([[
            pose.position.x,
            pose.position.y,
            pose.position.z
        ] for pose in pose_array.poses])

        # interpolate those points
        y = self.interpolate(y)

        # convert to Path
        path = Path()
        for i in range(y.shape[0]):
            pose_stamped = PoseStamped()
            pose = Pose()
            point = Point()
            point.x, point.y, point.z = y[i,:]
            pose.position = point
            pose_stamped.pose = pose
            path.poses.append(pose_stamped)
        return path

    def interpolate(self, numpy_array):
        raise NotImplementedError

    def test(self):

        # initialise plot
        fig, ax = plt.subplots(1)

        # make random 3D waypoints (but, only plot 3D)
        # y = np.array([
        #     [0, -1, 0],
        #     [0, 0, 0],
        #     [0, 1, 0],
        #     [1, 0, 0],
        #     [0, 0, 0],
        #     [-1, 0, 0],
        #     [0, -1, 0],
        #     [0, 0, 0]
        # ])
        y = np.array([
            [0, 0, 0],
            [0, 1, 0],
            [1, 1, 0],
            [1, 0, 0],
            [2, 0, 0],
            [2, 1, 0],
            [3, 1, 0],
            [3, 0, 0]
        ])
        ax.plot(y[:,0], y[:,1], 'k.')
        for p, n in zip(y, range(y.shape[0])):
            ax.annotate(n, (p[0], p[1]))

        # interpolate
        y = self.interpolate(y)
        ax.plot(y[:,0], y[:,1], 'k--', label='Interpolant')

        # formating
        ax.set_xlabel('$x$')
        ax.set_ylabel('$y$')
        ax.set_aspect('equal')
        ax.legend()

        # save the figure
        fig.savefig('../img/example_{}.png'.format(self.name), bbox_inches='tight', dpi=1000)
