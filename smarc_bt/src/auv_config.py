#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

"""
A simple config object for an AUV, to make sure that
the fields are common.
"""

import rospy

class AUVConfig(object):
    """
    Base config object, with default values for SAM.
    """
    def __init__(self):
        self.robot_name = 'sam'

        # topics
        self.DVL_TOPIC = 'core/dvl'
        self.LEAK_TOPIC = 'core/leak'
        self.CAMERA_DETECTION_TOPIC = 'detection/poi_down'
        self.PATH_TOPIC = 'ctrl/planned_path'
        self.PLAN_VIZ_TOPIC = 'viz/mission_waypoints'
        self.PLAN_PATH_TOPIC = 'ctrl/mission_waypoints'
        self.LATLON_TOPIC = 'dr/lat_lon'
        self.GPS_TOPIC = 'core/gps'
        self.ROLL_TOPIC = '/'+self.robot_name+'/dr/roll'
        self.PITCH_TOPIC = '/'+self.robot_name+'/dr/pitch'
        self.YAW_TOPIC = '/'+self.robot_name+'/dr/yaw'

        self.EMERGENCY_TOPIC = 'core/abort'
        self.HEARTBEAT_TOPIC = 'core/heartbeat'
        self.MISSION_COMPLETE_TOPIC = 'core/mission_complete'

        # Nacho's reloc topics
        self.RELOC_ENABLE_TOPIC = 'localization/enable'
        self.RELOC_WP = 'localization/wp'
        # Zheng and Li's algae farm thing
        self.ALGAE_FOLLOW_ENABLE_TOPIC = 'algae_farm/enable'
        self.ALGAE_FOLLOW_WP = 'algae_farm/wp'

        # actions and services
        self.ACTION_NAMESPACE = 'ctrl/goto_waypoint'
        self.EMERGENCY_ACTION_NAMESPACE = 'ctrl/emergency_surface_action'
        self.FOLLOW_ACTION_NAMESPACE = 'ctrl/leader_follower_action'
        self.START_STOP_DVL_NAMESPACE = 'core/toggle_dvl'
        self.INSPECTION_ACTION_NAMESPACE = 'ctrl/panoramic_inspection_action'
        self.PLANNED_SURFACE_ACTION_NAMESPACE = 'ctrl/planned_surface_action'


        self.LATLONTOUTM_SERVICE = '/'+self.robot_name+'/dr/lat_lon_to_utm'
        # in cases where the above service couldnt be found for some reason
        self.LATLONTOUTM_SERVICE_ALTERNATIVE = '/'+self.robot_name+'/lat_lon_to_utm'

        # tf frame names
        self.BASE_LINK = self.robot_name+'/base_link'
        self.UTM_LINK = 'utm'
        self.LOCAL_LINK = 'map'
        # this should be given as /sam/camera_down_link  or sam_1/camera_down_link etc.
        self.POI_DETECTOR_LINK = self.robot_name+'/camera_down_link'
        # some other link that is not THIS robot
        self.LEADER_LINK = 'sam_1/base_link'
        self.ENABLE_LEADER_FOLLOWER = True

        # imc related stuff, most likely never changes
        self.PLANDB_TOPIC = 'imc/plan_db'
        self.PLAN_CONTROL_TOPIC = 'imc/plan_control'
        self.ESTIMATED_STATE_TOPIC = 'imc/estimated_state'
        self.PLAN_CONTROL_STATE_TOPIC = 'imc/plan_control_state'
        self.VEHICLE_STATE_TOPIC = 'imc/vehicle_state'
        self.ABORT_TOPIC = 'imc/abort'
        self.GPSFIX_TOPIC = 'imc/gps_fix'
        self.GPS_NAV_DATA_TOPIC = 'imc/gps_nav_data'

        # hard values
        self.MAX_DEPTH = 20
        self.MIN_ALTITUDE = 1
        self.ABSOLUTE_MIN_ALTITUDE = -1
        # how many ticks to run emergency action before we give up
        # on the current wp and skip it
        # in ticks
        self.EMERGENCY_TRIALS_BEFORE_GIVING_UP = 30
        self.MIN_DISTANCE_TO_LEADER = 5
        # in seconds, dont spam the dvl service
        self.DVL_COOLDOWN = 0.5
        # how deep do we want to be to run the dvl
        # we wanna shut off the dvl on the surface
        self.DVL_RUNNING_DEPTH = 0.55

        # in meters
        self.WAYPOINT_TOLERANCE = 1.5

        # coverage planning variables
        # total width of sensor footprint, perpendicular to movement
        self.SWATH = 20
        # function of distance traveled. 0.01 means 1 meter error per 100m travel
        self.LOCALIZATION_ERROR_GROWTH = 0.02

        # Algae farm
        self.BUOY_TOPIC = 'sim/marked_positions'

        # Mission logging file location
        self.MISSION_LOG_FOLDER = '~/MissionLogs/'
        self.ENABLE_MANUAL_MISSION_LOG = False

        # lolo-specific
        self.LOLO_ELEVATOR_TOPIC = '/lolo/core/elevator'
        self.LOLO_ELEVON_PORT_TOPIC = '/lolo/core/elevon_port_fb'
        self.LOLO_ELEVON_STRB_TOPIC = '/lolo/core/elevon_strb_fb'
        self.LOLO_AFT_TANK_TOPIC = '/lolo/core/vbs/aft_tank_fb'
        self.LOLO_FRONT_TANK_TOPIC = '/lolo/core/vbs/front_tank_fb'

    def __str__(self):
        s = '\nAUV_CONFIG:\n'

        topics = 'TOPICS:\n'
        tflinks = 'TF LINKS:\n'
        actions_services = 'ACTIONS AND SERVICES:\n'
        others = 'OTHER:\n'
        for k,v in vars(self).items():
            l = '\t'+str(k)+'\t: '+str(v) +'\n'
            if 'TOPIC' in k:
                topics += l
            elif 'LINK' in k:
                tflinks += l
            elif 'ACTION' in k or 'SERVICE' in k:
                actions_services += l
            elif k == 'robot_name':
                continue
            else:
                others += l

        s += 'ROBOT_NAME:'+self.robot_name+'\n\n'
        s += topics
        s += tflinks
        s += actions_services
        s += others
        s += '********\n'
        return s


    def generate_launch_file(self, launchfile_path):
        def make_arg(name, default):
            if type(default) == type(self.robot_name):
                default = default.replace(self.robot_name+'/', '$(arg robot_name)/')
            return '\t<arg name="{}" default="{}" />\n'.format(name.lower(), default)

        def make_param(name):
            return '\t\t<param name="{}" value="$(arg {})" />\n'.format(name.lower(), name.lower())

        args_part = '\t<arg name="robot_name" default="sam" />\n\n'
        params_part ='\n\n\t<node name="smarc_bt" pkg="smarc_bt" type="smarc_bt.py" output="screen" ns="$(arg robot_name)">\n'

        for k,v in vars(self).items():
            if k == 'robot_name':
                params_part += make_param(k)
                continue
            else:
                args_part += make_arg(k,v)
                params_part += make_param(k)

        params_part += '\t</node>\n'

        generated_launchfile = ''
        generated_launchfile+= '<!-- THIS LAUNCH FILE WAS AUTO-GENERATED FROM src/auv_config.py, DO NOT MODIFY -->\n\n'
        generated_launchfile+= '<launch>\n'
        generated_launchfile+= args_part
        generated_launchfile+= params_part
        generated_launchfile+= '</launch>\n'

        with open(launchfile_path, 'w+') as f:
            f.write(generated_launchfile)
        print("Generated default launch file at {}".format(launchfile_path))
        print("You might need to restart the launch script (mission.launch) to read the new launch file")


    def read_rosparams(self):
        for k,v in vars(self).items():
            if v is not None:
                param_name = '~'+k.lower()
            else:
                param_name = k.lower()

            rosparam_v = rospy.get_param(param_name, None)
            if rosparam_v is not None:
                self.__dict__[k] = rosparam_v
            else:
                rospy.logwarn("{} parameter not read from launch file, using:{}".format(param_name, v))

            assert not (v is None and rosparam_v is None), "A required global rosparam ({}) is not set!".format(param_name)







