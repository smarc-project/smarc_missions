^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bt_mission
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2022-03-30)
------------------
* Removed launch of buoy publisher in mission.launch
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* - Added buoy marker publisher to mission launch file (I know it doesn't truly belong there).
  - Made buoy marker blackboard publisher and added it to the data ingestion tree.
* Merge pull request `#50 <https://github.com/smarc-project/smarc_missions/issues/50>`_ from svbhat/noetic-devel
  added planned surfacing and inspection actions.integrated surfacing i…
* Merge branch 'noetic-devel' into noetic-devel
* added planned surfacing and inspection actions.integrated surfacing in BT. Added separate action launch file.
* Merge branch 'noetic-devel' of https://github.com/smarc-project/smarc_missions into noetic-devel
* Update package.xml
* Merge pull request `#13 <https://github.com/smarc-project/smarc_missions/issues/13>`_ from smarc-project/noetic-devel
  Update smarc_missions to latest version consistent with DR implementation
* Merge branch 'noetic-devel' into freshness_checks
* Merge pull request `#42 <https://github.com/smarc-project/smarc_missions/issues/42>`_ from smarc-project/fix_sam_action_servers_dep
  Fix sam action server dep
* Update package.xml
* Update package.xml
* Merge pull request `#40 <https://github.com/smarc-project/smarc_missions/issues/40>`_ from svbhat/noetic-devel
  Updates to action servers and the BT with dynamic dynamic reconfigure
* Merge branch 'noetic-devel' of https://github.com/svbhat/smarc_missions into noetic-devel
* Launched actions before the BT so the BT can reach them.
* Updated action servers to work with new topics and Stonefish.
* Renamed goto_waypoint action to be consistent with naming
* Commented out controllers from bt launch file. Controllers launched separately.
* added low level controllers to mission.launch
* Included new actions
* Merge pull request `#11 <https://github.com/smarc-project/smarc_missions/issues/11>`_ from smarc-project/noetic-devel
  Updating to latest version of smarc_missions
* Merge pull request `#37 <https://github.com/smarc-project/smarc_missions/issues/37>`_ from smarc-project/python3-install
  Update CMakeLists.txt
* Update package.xml
* Merge pull request `#34 <https://github.com/smarc-project/smarc_missions/issues/34>`_ from KKalem/noetic-devel
  Changes from lolo tests
* Changes from lolo tests
* Merge pull request `#33 <https://github.com/smarc-project/smarc_missions/issues/33>`_ from KKalem/noetic-devel
  Noetic devel
* Merge branch 'noetic-devel' of https://github.com/KKalem/smarc_missions into noetic-devel
* Update mission.launch
* Update mission.launch
* Merge branch 'noetic-devel' of https://github.com/KKalem/smarc_missions into noetic-devel
* Merge branch 'noetic-devel' into noetic-devel
* Merge pull request `#32 <https://github.com/smarc-project/smarc_missions/issues/32>`_ from smarc-project/test_mission
  Add smarc_mission_sim and mission test
* Fixed more deps in package.xml
* Launch file configs and small fixes
* Added install rules and missing deps in package xml
* Removed unneeded param from mission.launch
* Lolo debugs from lolo with love
* Fixed the launch file arg name changes
* Fixed launch files
* removed cbf stuff that started to error now for some reason
* Change package sam_mission to bt_mission
* Contributors: Nils Bore, Ozer, Ozer Ozkahraman, Sriharsha Bhat, cisprague, svbhat, Özer Özkahraman
