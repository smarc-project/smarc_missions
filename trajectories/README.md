# Trajectories package

This will hopefully eventually contain a plethora of different trajectory types, the first implemented being `interp1d`.

Call the service using `rosservice call /interp1d` and launch the node using `roslaunch trajectories interp1d.launch`. See the arguments in the launch file.

An example with `spline_degree = 'cubic'` and `n_points = 1000` looks like this:
![Cubic spline](img/example_interp1d.png)

The solid line represents the "coarse" or "wished" waypoints, coming from the operator (or potentially autonomy), and dashed represents the interpolant (in this case a cubic spline).