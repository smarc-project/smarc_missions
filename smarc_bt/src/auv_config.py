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
                val = get_param(k)
                if val is not None:
                    # with values coming from rosparam server if they are there
                    # we assume this requires no further processing
                    self.__dict__[k] = val
                else:
                    # or from the yaml file if not in rosparams
                    # in this case, services and base link
                    # specifically need the robot name pre-prended
                    # we assumed robot_name is the first thing in the dict always
                    # and that it has already been interned
                    if k in ['LATLONTOUTM_SERVICE',
                             'UTM_TO_LATLON_SERVICE',
                             'LATLONTOUTM_SERVICE_ALTERNATIVE',
                             'BASE_LINK']:
                        # slight idiot-proofing
                        if val[0] != '/':
                            val = '/' + self.robot_name + '/' + val
                        else:
                            val = '/' + self.robot_name +  val

