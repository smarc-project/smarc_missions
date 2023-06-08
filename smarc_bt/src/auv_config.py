#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

"""
A simple config object for an AUV, to make sure that
the fields are common.
"""

import rospy, yaml

def get_param(name, default=None):
    return rospy.get_param(rospy.search_param(name), default)

class AUVConfig(object):
    """
    Base config object, with default values for SAM.
    """
    def __init__(self):
        self._read_rosparams()
        self._handle_robot_name_subs()

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



    def _read_rosparams(self):
        # read the config.yaml file in the launch directory to construct this object
        # the file location itself is a launch arg
        configyaml = get_param("config_file")
        # read that yaml file
        # and then internalize its fields only
        # the values will come from rosparams
        with open(configyaml, 'r') as f:
            # config is a smple dict
            config = yaml.safe_load(f)
            for k,v in config.items():
                # and we want its keys to be the object fields
                # if rosparam for some reason doesnt have the
                # key, then we use the value in yaml and hope for the best
                self.__dict__[k] = get_param(k, v)

    def _handle_robot_name_subs(self):
        # services need to look like /$(arg robot_name)/service_name
        # so add the robot_name part
        for k in ['LATLONTOUTM_SERVICE',
                  'UTMTOLATLON_SERVICE',
                  'GOTO_ACTION_NAMESPACE',
                  'EMERGENCY_ACTION_NAMESPACE']:
            self.__dict__[k] = '/' + self.robot_name + '/' + self.__dict__[k]

        # and base link needs $(arg robot_name)/base_link
        # or other TF names that depend on robot name
        for k in ['BASE_LINK']:
            self.__dict__[k] = self.robot_name + '/' + self.__dict__[k]

