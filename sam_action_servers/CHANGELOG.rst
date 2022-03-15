^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sam_action_servers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added unplanned -live- waypoints that can be followed in the middle of a mission
* Merge branch 'noetic-devel' into noetic-devel
* Merge pull request `#66 <https://github.com/smarc-project/smarc_missions/issues/66>`_ from svbhat/noetic-devel
  Added ddynamic reconfig for lookahead distance. Take wp tolerance fro…
* Added ddynamic reconfig for lookahead distance. Take wp tolerance from neptus.
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* Merge pull request `#64 <https://github.com/smarc-project/smarc_missions/issues/64>`_ from svbhat/noetic-devel
  Updated WP action to take RPMs from Neptus
* Updated WP action to take RPMs from Neptus
* Merge branch 'noetic-devel' of https://github.com/KKalem/smarc_missions into noetic-devel
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* Merge pull request `#63 <https://github.com/smarc-project/smarc_missions/issues/63>`_ from svbhat/noetic-devel
  Cleaned up wp action, removed TimerCallback. Added timeout. Uodated turbo-turn logic.
* Cleaned up wp action. Added timeout. Uodated turbo-turn logic.
* Merged with noetic-devel
* Merge pull request `#59 <https://github.com/smarc-project/smarc_missions/issues/59>`_ from KKalem/noetic-devel
  End-of-mission surfacing, manual logging
* Merge pull request `#59 <https://github.com/smarc-project/smarc_missions/issues/59>`_ from KKalem/noetic-devel
  End-of-mission surfacing, manual logging
* Added planned surfacing node connected to BT.
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into mock_sensor_reader
* Merge pull request `#56 <https://github.com/smarc-project/smarc_missions/issues/56>`_ from svbhat/noetic-devel
  Added parameter flags to the wp planner to enable or disable extra fe…
* Minor update to abstract out overshot WP condition.
* Added parameter flags to the wp planner to enable or disable extra features
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into devel
* Merge pull request `#54 <https://github.com/smarc-project/smarc_missions/issues/54>`_ from svbhat/noetic-devel
  Attempts to address missing waypoints and overshooting.
* Added a logic to handle overshooting the waypoint with crosstrack error.
* Abstracted controller toggle services, cleaned up diving logic, added vbs.
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* Merge pull request `#50 <https://github.com/smarc-project/smarc_missions/issues/50>`_ from svbhat/noetic-devel
  added planned surfacing and inspection actions.integrated surfacing i…
* Merge branch 'noetic-devel' into noetic-devel
* added planned surfacing and inspection actions.integrated surfacing in BT. Added separate action launch file.
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* Merge pull request `#47 <https://github.com/smarc-project/smarc_missions/issues/47>`_ from svbhat/noetic-devel
  Added a correction to read the depth waypoint while checking WP success
* Added a turbo-turn inspection action, updated emergency action to prevent random enabling of controllers.
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* updated controller topics to common republished topic from services
* Used the new common control topics for control!
* Updated actions to include controller toggle services.
* Kind-of working crosstrack error.
* Added velocity control based on velocity from Neptus!
* adding crosstrack flag.
* Adding crosstrack error, not yet working
* adding crosstrack error, not yet working.
* Added a correction to read the depth waypoint while checking feedback to see if waypoint is reached.
* Merge pull request `#46 <https://github.com/smarc-project/smarc_missions/issues/46>`_ from svbhat/noetic-devel
  Corrected the issue that SAM does not dive during the mission due to a wrong depth goal.
* Corrected depth setpoint from goto_waypoint goal.
* Update package.xml
* Merge pull request `#45 <https://github.com/smarc-project/smarc_missions/issues/45>`_ from svbhat/noetic-devel
  Updated go to waypoint action to publish the correct yaw setpoint.
* Updated published yaw setpoint to be consistent with DR coordinates.
* Merge pull request `#40 <https://github.com/smarc-project/smarc_missions/issues/40>`_ from svbhat/noetic-devel
  Updates to action servers and the BT with dynamic dynamic reconfigure
* Merge pull request `#2 <https://github.com/smarc-project/smarc_missions/issues/2>`_ from svbhat/noetic-devel
  Topic changes from Harsha
* updated yaw feedback topic to fit naming convention
* updated latest naming convention for yaw feedback topic
* Merge pull request `#1 <https://github.com/smarc-project/smarc_missions/issues/1>`_ from svbhat/noetic-devel
  Harsha's changes
* Updates to dependencies to make the packages build.
* Merge pull request `#12 <https://github.com/smarc-project/smarc_missions/issues/12>`_ from KKalem/noetic-devel
  Merging Ozer's dynamic reconfigure additions
* updated dependencies
* Added version to sam_action_servers package.xml and install python to cmakelists
* Added dependencies to package.xml
* Merge branch 'noetic-devel' of https://github.com/svbhat/smarc_missions into noetic-devel
* Updated action servers to work with new topics and Stonefish.
* changed rpm datatype
* Renamed goto_waypoint action to be consistent with naming
* Updated action types to GotoWaypointAction
* updated wp_depth_action_planner
* updated wp_depth_action_planner
* updated install rules for sam_action_servers
* Updates to wp_depth_action_planner.
* replaced all instances of auv_simple_motion_planner
* Corrected indentation in wp_depth_action_planner.
* changed name of sam_actions to sam_action_servers. Fixed indentation errors.
* Contributors: Jollerprutt, Nils Bore, Ozer Ozkahraman, Sriharsha Bhat, cisprague, svbhat, Özer Özkahraman
