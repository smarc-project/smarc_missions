^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package smarc_bt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Debug
* Added live-updating wp following for the algae farm scenario.
* Added retry to emergency surface as well
* Debugged logging
* Added unplanned -live- waypoints that can be followed in the middle of a mission
* Merge pull request `#71 <https://github.com/smarc-project/smarc_missions/issues/71>`_ from KKalem/noetic-devel
  Added target wp marker visuals and limited rviz plan viz to 0m height
* py2 math.dist doesnt exist apparently
* Added target wp marker visuals and limited rviz plan viz to 0m height
* Merge branch 'noetic-devel' into new_ci
* Merge pull request `#69 <https://github.com/smarc-project/smarc_missions/issues/69>`_ from KKalem/noetic-devel
  Logging improvements, lack of initial gps fix is now a blocking condition for the BT
* Added casting to reconfig to handle ints being passed to float variables as params
* Merge branch 'noetic-devel' into new_ci
* Fixed gps time, added swath plotting
* stopped logging stale gps fixes, made lack of initial gps fix a fail condition for the BT
* Limited coverage planning so it doesnt generate _backwards\_ plans
* Merge pull request `#68 <https://github.com/smarc-project/smarc_missions/issues/68>`_ from KKalem/noetic-devel
  Better logs, no change to floats
* Debugged log viewer, went back to float64s for r/p/y
* Float64 to FloatStamped
* Merge pull request `#65 <https://github.com/smarc-project/smarc_missions/issues/65>`_ from KKalem/noetic-devel
  Lolo-specific logging, gps logging to neptus, plotting
* Fixed none gps?
* Merge branch 'noetic-devel' into noetic-devel
* Merge pull request `#67 <https://github.com/smarc-project/smarc_missions/issues/67>`_ from smarc-project/revert-62-mock_sensor_reader
  Revert "Spiral pattern and mock buoy detection"
* Revert "Spiral pattern and mock buoy detection"
* Changed the behaviour of the stop button to finalize a mission instead of pausing
* More logging, better plots, gps fixed
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* more gps log fixes
* Fixed gps logging
* limiting mission plan plotting to 0m, added stat/end texts, added the read gps node to tree
* Merge branch 'noetic-devel' of https://github.com/KKalem/smarc_missions into noetic-devel
* Added time ros time logging instead of system time
* lolo changes?
* Merge branch 'noetic-devel' of https://github.com/KKalem/smarc_missions into noetic-devel
* Merge pull request `#62 <https://github.com/smarc-project/smarc_missions/issues/62>`_ from cisprague/mock_sensor_reader
  Spiral pattern and mock buoy detection
* Prepare for destruction
* Added lolo-specific logging and infra for sam-specific logging too
* Changed depends
* Added conditional dependency of sklearn
* Added sklearn dependency and explicitly wrote latlon centroid of prototype
* Merge pull request `#61 <https://github.com/smarc-project/smarc_missions/issues/61>`_ from KKalem/noetic-devel
  Added rviz visualizations for auv path, mission waypoints, bottom fro…
* Added rectangle spiral plan
* Added rviz visualizations for auv path, mission waypoints, bottom from dvl
* Merge pull request `#60 <https://github.com/smarc-project/smarc_missions/issues/60>`_ from KKalem/noetic-devel
  Added ro pi yaw altitude time tracing to log
* Merged with noetic-devel
* Square plan made rectangular
* Added ro pi yaw altitude time tracing to log
* Merge pull request `#59 <https://github.com/smarc-project/smarc_missions/issues/59>`_ from KKalem/noetic-devel
  End-of-mission surfacing, manual logging
* Merge pull request `#59 <https://github.com/smarc-project/smarc_missions/issues/59>`_ from KKalem/noetic-devel
  End-of-mission surfacing, manual logging
* Merge branch 'noetic-devel' of https://github.com/KKalem/smarc_missions into noetic-devel
* Added the return from finalize
* Merge branch 'noetic-devel' of https://github.com/KKalem/smarc_missions into noetic-devel
* Forgot the empty
* Merge branch 'noetic-devel' of https://github.com/KKalem/smarc_missions into noetic-devel
* Made mission finalzer publish once per mission
* Merge branch 'noetic-devel' of https://github.com/KKalem/smarc_missions into noetic-devel
* Added manual logging, enable/disable via reconfig
* Localisation trajectory working in sim
* Merge pull request `#58 <https://github.com/smarc-project/smarc_missions/issues/58>`_ from KKalem/noetic-devel
  quick and dirty mission logging
* Merge pull request `#58 <https://github.com/smarc-project/smarc_missions/issues/58>`_ from KKalem/noetic-devel
  quick and dirty mission logging
* Fixed mission log folder naming convention error
* Fixed py2/3 exit_ok problem, made log save location dynamic reconfigable
* To ssh
* Continue tomorrow
* Finished buoy localisation prototype
* Made z axis equal too, shebang
* quick and dirty mission logging
* Added notebooks of buoy prototypes
* Merge pull request `#57 <https://github.com/smarc-project/smarc_missions/issues/57>`_ from KKalem/noetic-devel
  Made waypoint tolerance dynamic reconfigable
* Merge pull request `#57 <https://github.com/smarc-project/smarc_missions/issues/57>`_ from KKalem/noetic-devel
  Made waypoint tolerance dynamic reconfigable
* Remembered my self
* Made waypoint tolerance dynamic reconfigable
* Merge pull request `#52 <https://github.com/smarc-project/smarc_missions/issues/52>`_ from cisprague/devel
  Updates related to buoy position reading and publishing.
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* Added buoy planner
* Added visualization dependency to CMakeLists.txt and package.xml
* - Removed edit from BT launch file.
  - Publushed None to Blackboard if no buoys.
* Merge branch 'smarc-project:noetic-devel' into noetic-devel
* Updated buoy reader to be more robust
* Updated buoy reader
* Merge pull request `#53 <https://github.com/smarc-project/smarc_missions/issues/53>`_ from KKalem/noetic-devel
  Neptus GpsFix, CoverArea maneuver, Coverage path planning
* Small debugging
* Made the coverage plan try to start as close to the CoverArea waypoit itself
* Added cover area automatic coverage planning v0
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* - Added buoy marker publisher to mission launch file (I know it doesn't truly belong there).
  - Made buoy marker blackboard publisher and added it to the data ingestion tree.
* Added gps fix and gps nav data for neptus
* Merge pull request `#51 <https://github.com/smarc-project/smarc_missions/issues/51>`_ from KKalem/noetic-devel
  Added first version of cover area handling, going tru poly points for…
* Added first version of cover area handling, going tru poly points for now
* Merge pull request `#50 <https://github.com/smarc-project/smarc_missions/issues/50>`_ from svbhat/noetic-devel
  added planned surfacing and inspection actions.integrated surfacing i…
* Rolled back BT surfacing logic. Removed action clients, reused the GOTOWAYPOINT action instead.
* Merge branch 'noetic-devel' into noetic-devel
* added planned surfacing and inspection actions.integrated surfacing in BT. Added separate action launch file.
* Merge pull request `#49 <https://github.com/smarc-project/smarc_missions/issues/49>`_ from KKalem/noetic-devel
  Mission finalization, re-running changes. Reduced service timeout.
* Made re-doing missions easier, fixed edge cases with accepting missions, added small animation to  simple publisher
* Small name change
* Changed initial waiting for latlontoutm to 0.5s from 10s
* Merge pull request `#48 <https://github.com/smarc-project/smarc_missions/issues/48>`_ from KKalem/noetic-devel
  Added a backup service name to latlontoutm
* Added a backup service name to latlontoutm stuff because spaghetti
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* checks for some ros logging methods
* Update package.xml
* Merge pull request `#13 <https://github.com/smarc-project/smarc_missions/issues/13>`_ from smarc-project/noetic-devel
  Update smarc_missions to latest version consistent with DR implementation
* Merge pull request `#43 <https://github.com/smarc-project/smarc_missions/issues/43>`_ from KKalem/noetic-devel
  Fixes lat_lon_to_utm service, new feedbacks in tree-watcher, data freshness beginnings
* Added handling of None alt and depth values, updated lat_lon_to_utm service name
* Merge branch 'noetic-devel' into freshness_checks
* added bt_common to viminorder
* Fixed ghost-paste, MOAR FEEDBACKS
* Added time limitations to read_topic action so that it can fail when there is no data or data is late
* Added time logging to reading/checking actions
* Merge pull request `#42 <https://github.com/smarc-project/smarc_missions/issues/42>`_ from smarc-project/fix_sam_action_servers_dep
  Fix sam action server dep
* Update package.xml
* Merge pull request `#40 <https://github.com/smarc-project/smarc_missions/issues/40>`_ from svbhat/noetic-devel
  Updates to action servers and the BT with dynamic dynamic reconfigure
* Added  ddynamic_reconfig_python to package.xml of smarc_bt
* Merge pull request `#1 <https://github.com/smarc-project/smarc_missions/issues/1>`_ from svbhat/noetic-devel
  Harsha's changes
* Removed smarc_bt/cfg directory
* Updates to dependencies to make the packages build.
* explicitly stated dependencies to check it that helps with CI
* Merge pull request `#12 <https://github.com/smarc-project/smarc_missions/issues/12>`_ from KKalem/noetic-devel
  Merging Ozer's dynamic reconfigure additions
* Added dynamic reconfig to BT, alt and depth now checks from BB
* Adding dynamic reconfig for BT parameters
* Merge pull request `#11 <https://github.com/smarc-project/smarc_missions/issues/11>`_ from smarc-project/noetic-devel
  Updating to latest version of smarc_missions
* Added switching actions for wp types, changed some printing and feedback to be more pleasant to eyes
* Merge pull request `#38 <https://github.com/smarc-project/smarc_missions/issues/38>`_ from KKalem/noetic-devel
  New "sample" maneuver and fixed mission finalization.
* Added a readme
* QOL changes. Better checks for changes in plan
* Simplified run tree, added a couple QOL stuff. The BT will now unfinalize a mission if a new one is sent
* Changed shebang and prints to logwarns
* Added handling for SAMPLE maneuver, fixed some printing bugs, fixed mission finalization.
* Extracted latlontoutm as a function in mission plan, fixed(?) the rospy.logwarn that was incorrectly formatted
* Made abort message and abort action into a sequence so both are done all the time
* Merge pull request `#37 <https://github.com/smarc-project/smarc_missions/issues/37>`_ from smarc-project/python3-install
  Update CMakeLists.txt
* Update package.xml
* Update CMakeLists.txt
* Merge pull request `#34 <https://github.com/smarc-project/smarc_missions/issues/34>`_ from KKalem/noetic-devel
  Changes from lolo tests
* typo...
* Tidied up mission finalization and captain hand-off, some printing
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* Fixed param bugs, somehow the one line that added the param declarations to the launchfile had vaporized
* Merge pull request `#36 <https://github.com/smarc-project/smarc_missions/issues/36>`_ from nilsbore/fix_tree_file
  Fixed install problem with writing last_ran_tree in installed path
* Fixed install problem with writing last_ran_tree in installed path
* Changes from lolo tests
* Merge pull request `#33 <https://github.com/smarc-project/smarc_missions/issues/33>`_ from KKalem/noetic-devel
  Noetic devel
* Added the missing NOT...
* Print on plan complete
* Merge branch 'noetic-devel' of https://github.com/KKalem/smarc_missions into noetic-devel
* fixed typo
* Merge branch 'noetic-devel' into noetic-devel
* Merge pull request `#32 <https://github.com/smarc-project/smarc_missions/issues/32>`_ from smarc-project/test_mission
  Add smarc_mission_sim and mission test
* Removed iteritems to make it work with python3
* Launch file configs and small fixes
* Added install rules and missing deps in package xml
* Hopefully fixed the mission_complete thing
* Merge remote-tracking branch 'origin/noetic-devel' into noetic-devel
* Separated plan_id for no plan and mission complete for CI purposes
* Merge pull request `#30 <https://github.com/smarc-project/smarc_missions/issues/30>`_ from NiklasRolleberg/lolo_test
  small bug fixes
* small bug fixes
* Small cleanup for parameters
* Added the ability to get z units, speed and speed units from neptus
* Seeting goal tolerance from action server
* topic changes for abort, heartbeat, mission complete and lat_lon_to_utm service. Improved luanchfile generator
* Lolo debugs from lolo with love
* Removed msg generation from cmake
* Fixed the launch file arg name changes
* Fixed launch files
* removed cbf stuff that started to error now for some reason
* Change sam_bt -> smarc_bt
* More changes to the new interfaces
* More movement to common interfaces, added utm zone/band global rosparams requirement
* Merge branch 'action_mods' into noetic-devel
* Action client mods and other general changes for new interfaces
* Merge pull request `#26 <https://github.com/smarc-project/smarc_missions/issues/26>`_ from iyuner/noetic-devel
  Add report mission complete in the end of the tree
* add report mission complete in the end of the tree
* Merge pull request `#25 <https://github.com/smarc-project/smarc_missions/issues/25>`_ from iyuner/noetic-devel
  add bt's heartbeat message to publish
* correct heartbeat topic name
* run it, change topic name, and auto-generate some files
* add bt heart beat to publish
* Merge pull request `#23 <https://github.com/smarc-project/smarc_missions/issues/23>`_ from smarc-project/kristineberg2011
  Changed so that bt reads utm zone and band from parameters set when w…
* Changed so that bt reads utm zone and band from parameters set when we start system
* Solved the path to the tree with a hack
* Update package.xml
* Merge pull request `#10 <https://github.com/smarc-project/smarc_missions/issues/10>`_ from smarc-project/master
  Updating latest version
* Better depth handling for wps in mission plan obj
* Removed secondary emergency surfacing
* Removed automatic DVL toggling, nothing is stable enough for this to work yet
* Changed dvl threshold
* Merge branch 'master' into master
* Removed unused file, updated launch file
* Added more feedback messages, use py-trees-tree-watcher to see them
* Forgotten arg, removed tree status spam
* Removed many logs, made altOK consider the initial altitude which might be already too shallow
* Moved the tf listener inside the MissionPlan object to the static method, seems to have fixed the blackboard shenanigans?
* More debugging the blackboard shenanigans
* made utm setter succeed all the time
* typos
* Setup returns
* moved subs and pubs into setup for all actions and conds
* more of the same....
* more logging for plandb stuff
* show status
* more
* more shenanigans...
* removed wait for xform from mission plan init, plandb shenanigans continues
* typo
* moved plandb handling to update from subscriber callback, hopefully this will fix the blackboard shenanigans?
* changed dvl on off service name
* Merge branch 'master' of https://github.com/smarc-project/smarc_missions
* Changed dvl start stop service default
* Merge branch 'master' of https://github.com/smarc-project/smarc_missions
* Made altOK return success when there is no altitude too
* Merge branch 'master' of https://github.com/smarc-project/smarc_missions
* changed topics, added dvl toggling based on depth
* Added robustness against path planner service not existing too, fixed print bug in follower action
* Made BT able to run even when NOTHING is running. Added 'forceful' emergency surfacing in the cases that the emergency surfacing action server can not be used
* Merge branch 'master' of https://github.com/smarc-project/smarc_missions
* Debugging
* Merge branch 'master' of https://github.com/smarc-project/smarc_missions
* debugging blackboard shenanigans
* Added none check to set next plan action
* Added try catch around tree.tick()
* removed feedback_cb bb setting
* merge
* First working version of leader-follower
* Made auto generation of launch file more portable, so long as nobody uses LISP with ROS
* Restructred tree to allow for extra actions if no plan is available, started with leader follower
* Added leader follower action, fixed its indentation
* Added leader follower subtree
* Merge pull request `#9 <https://github.com/smarc-project/smarc_missions/issues/9>`_ from smarc-project/master
  Merging latest before leader-follower
* removed some spam
* Added autogeneration of bt_sam.launch, added follow link action, fixed camera link hardcoding
* Merge pull request `#12 <https://github.com/smarc-project/smarc_missions/issues/12>`_ from KKalem/master
  Fixed setup problem, added cbf to altitude, camera_link rosparam
* Added camera_link to bt_sam.launch
* Fixed setup problem, added cbf to altitude
* deleted md5 printing
* Corrected waypoints viz for rviz's point of view
* Added mission plan visualization for rviz
* Corner case when all wps are skipped due to emergency
* Added hacky path visulaization to Refine action
* quick fix, forgot my self
* Added subtree to give up on a waypoint if it triggers an emergency too many times. How many is an arg in launch
* Tied path planner distance to wp tolerance, fixed some corner cases with neptus feedback
* Improved feedback into Neptus. Now it includes tree tip, its status, and autonomy mode
* Made the upload button also disable autonomy
* Added autonomous plan subtree, added autonomy toggling from neptus, added POI inspection autonomous planning
* Added vehicle position as first wp to be refined
* reduced spam of wp_follower, added altitude checks to BT, added new launch params
* Bugfix and blackbox levels to help reduce clutter in rqt
* typo fix
* Moved path planner name to launch files
* Added option to disable the use of path planner
* Fixed assertion in interp1d, added path planner to bt, first working version
* Bugfixes, added plan change detection
* Restructing bt for asko
* Merge branch 'master' of https://github.com/smarc-project/smarc_missions
* Merge pull request `#8 <https://github.com/smarc-project/smarc_missions/issues/8>`_ from smarc-project/master
  Merge latest smarc_missions
* Changed max depth of sam to 20m because asko
* Disabled turbo turn by default until its fixed, fixed small import oversight in bt_actions
* Merge pull request `#9 <https://github.com/smarc-project/smarc_missions/issues/9>`_ from smarc-project/dual_ekf_test
  Dual ekf test
* Merge branch 'master' of https://github.com/smarc-project/smarc_missions into dual_ekf_test
* Upgraded cbfs:removed looping, made resetting not require a specific action in the tree
* Added CBFCOndiiton and the required messages for it. Made depth and altitude cbf conditions
* More disentanglement
* Disentangled the BT from SAM
* More waypoints to smarc_bt renames
* Renamed waypoints to smarc_bt in prep for the BT being used on lolo too
* Contributors: Niklas, Nils Bore, Ozer, Ozer Ozkahraman, Sriharsha Bhat, Torroba, cisprague, ignaciotb, svbhat, xyp8023, yiya, Özer Özkahraman
