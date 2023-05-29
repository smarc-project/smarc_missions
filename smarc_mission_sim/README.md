To test locally: `roslaunch fake_user.launch` launches all the nodes needed, and `./fake_nodered.py` will test the BT acting as as a fake neptus and user. Launch these in separate terminals to debug.

Launch sequence:
CMake -> test/fake_user.launch
- launch/mission_sim.launch
	- lolo_description.launch
	- odom_tf.launch
	- smarc_mission_sim/mission_sim_node
	- smarc_mission_sim/fake_hardware.py
	- smarc_bt/mission.launch
- fake_nodered.py
