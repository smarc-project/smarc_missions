auv_mission_planner
===================

This package contains a simple mission planner that is designed to be used with the
[`auv_sm_mission_executor`](https://github.com/smarc-project/smarc_planning/tree/master/auv_sm_mission_executor)
package but that can also be used as a standalone tool to create LoLo waypoint files.

## Installing

This package requires ros and catkin.
If you're unfamiliar with catkin, see [the tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

In your catkin workspace `src` folder, clone the containing package:
```
git clone https://github.com/smarc-project/smarc_planning.git
```
Now, you should be able to build the package as usual, using `catkin_make`.

## Dependencies

You will probably need to install the `geodesy` package: `sudo apt-get install ros-kinetic-geodesy`.

## Running

First, make sure that you have sourced your catkin workspace (`source /path/to/catkin_ws/devel/setup.bash`).
In the simplest case, the whole mission planner can then be run using
```
roslaunch auv_mission_planner mission_planner.launch
```
This should launch and rviz window.
To see satellite maps inside of rviz, you will need to replace the entry of the `Object URI` field as shown below.

![Aerial map display](https://raw.githubusercontent.com/smarc-project/smarc_planning/master/auv_mission_planner/resources/aerial_map_display.png)

You can either get the entry from @nilsbore or register at https://www.mapbox.com/ to
get an `access_token` to insert into the URL: `https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}.jpg?access_token=<access_token>`.

## Usage

Once you have launched the command above, you can add and delete new waypoints by right-clicking
the circles around the waypoints, as shown in the screenshot below.

![Context menu](https://raw.githubusercontent.com/smarc-project/smarc_planning/master/auv_mission_planner/resources/context_menu.png)

You can also change the position of the waypoints by dragging the circles with the left mouse button.

To modify the view of the interface, use the scroll wheel to zoom in and out. You can rotate
by holding the left mouse button and dragging the background. To translate the view to a new position,
press `shift` at the same time as dragging the background with the left mouse button.

By default, the mission file will be save in `~/.ros/mission.csv` and the exported LoLo waypoint file
in `~/.ros/mission.lolo`. You can change this to use other files, as described in the next section.
The mission itself can only be loaded from  a mission file (as opposed to a LoLo file),
so make sure to always save a mission file. You can continue to modify the plan once loaded, and save it again
when done.

## Arguments

You can supply several arguments to the launch file as shown below:
```
roslaunch auv_mission_planner mission_planner.launch mission_file:=/path/to/mission.csv default_rpm:=300 goal_tolerance:=50 latitude:=59.34 longitude:=18.14
```

* `mission_file` - in this case, saves to `/path/to/mission.csv` and will save LoLo waypoints to `/path/to/mission.lolo`
* `default_rpm` - the RPM to set in LoLo exports
* `goal_tolerance` - radius around goal
* `latitude`, `longitude` - this will be the area around which we plan our mission, maps will be automatically loaded if there is internet available



