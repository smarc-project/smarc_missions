#!/usr/bin/env python3

# Christopher Iliffe Sprague
# Parent class, describing a ROS service that takes
# a PoseArray of 'coarse waypoints' and returns a Path.
# Usage: 'imc(utm) -> bt -> poseArray(map) -> path_planner -> path(map) -> bt'

import rospy, numpy as np
from scipy.interpolate import interp1d

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from nav_msgs.msg import Path
from trajectories.srv import trajectory

class Interp1d:

    def __init__(self):

        # start node
        rospy.init_node('interp1d')
        rospy.Service('interp1d', trajectory, self.__call__)

        # ROS parameters from launch file
        self.spline_degree = rospy.get_param("~spline_degree")
        self.waypoint_spacing = rospy.get_param("~waypoint_spacing")

    def __call__(self, trajectory_request):

        # sanity
        pose_array = trajectory_request.coarse
        assert isinstance(pose_array, PoseArray), "Argument not a pose array, it is:"+str(type(pose_array))

        # convert PoseArray to numpy
        y = np.array([[
            pose.position.x,
            pose.position.y,
            pose.position.z
        ] for pose in pose_array.poses])

        # interpolate those points
        y = self.compute(y)

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

    def compute(self, numpy_array):

        # independent variable: distance between points
        x = np.array([numpy_array[i+1,:] - numpy_array[i,:] for i in range(numpy_array.shape[0] - 1)])
        x = np.linalg.norm(x, axis=1)
        x = np.hstack(([0], np.cumsum(x)))

        # interpolate the data
        print(self.spline_degree, self.waypoint_spacing)
        f = interp1d(x, numpy_array, kind=self.spline_degree, axis=0)
        x = np.arange(start=x[0], stop=x[-1], step=self.waypoint_spacing)
        y = f(x)
        return y

def test():

    # initialise plot
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1)

    # make random 3D waypoints (but, only plot 3D)
    n = 15
    y = np.random.uniform(low=-1, high=1, size=(n,3))
    ax.plot(y[:,0], y[:,1], 'k.-')

    # interpolate
    y = Interp1d().compute(y)
    ax.plot(y[:,0], y[:,1], 'k--')

    # save the figure
    fig.savefig('../img/example_interp1d.png', bbox_inches='tight')




if __name__ == "__main__":
    Interp1d()
    rospy.spin()
    # test()
