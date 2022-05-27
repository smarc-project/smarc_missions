#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import imc_enums, bb_enums, common_globals
from mission_plan import MissionPlan

import time, json, uuid, ssl, sys, os
import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as mqtt_sub

class MQTTHandler(object):
    """
    A class to handle all the connections and comms
    related to MQTT and the WARA-PS stuff
    Tested with Mosquitto broker
    """
    def __init__(self,
                 auv_config,
                 vehicle,
                 blackboard):

        self._vehicle = vehicle
        self._config = auv_config
        self._bb = blackboard

        # a list of messages from all the different parts of the handler
        self.feedback_messages = []

        self.robot_name = self._vehicle.robot_name
        self._uuid = uuid.uuid4().hex
        # one of: ground, air, surface, subsurface
        if 'sam' in self.robot_name or 'lolo' in self.robot_name:
            self._waraps_agent_type = "subsurface"
        else:
            self._waraps_agent_type = "surface"

        self._client = mqtt.Client(client_id = self.robot_name)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

        use_waraps = True
        if use_waraps:
            try:
                # XXX this will probably break with ROS shenanigans...
                secrets_file = os.path.expanduser('~/.waraps_broker_SECRET')
                with open(secrets_file, 'r') as f:
                    user, pw = f.readlines()
                    user = user.strip('\n')
                    pw = pw.strip('\n')
            except Exception as e:
                print(e)
                print("You should have a file named 'mqtt_SECRET' with the username and password of the waraps broker details in one line each")
                sys.exit(1)

            self._client.username_pw_set(user, pw)
            self._client.tls_set(cert_reqs=ssl.CERT_NONE)
            self._client.tls_insecure_set(True)
            self._client.connect('broker.waraps.org', 8883, 60)
            print("Using waraps broker")
        else:
            self._client.connect('localhost', 1884, 60)
            print("Using local broker")

        self._client.loop_start()




    def tick(self):
        self.feedback_messages = []
        self._tick_waraps()




    def _on_connect(self, client, userdata, flags, rc):
        """
        called when the client connection is successful
        fill in the topics list below to subscribe to all of them
        """
        self.feedback_messages.append("Connected with result code:{}".format(rc))

        # list of all the topics and their callbacks
        # ((topic, qos), callback_fn)
        topics = [
            (('{}/test'.format(self.robot_name), 0), self._handle_test)
        ]

        for (t, qos),cb in topics:
            self._client.message_callback_add(t, cb)

        # subscribe to all the topics at once
        # this magic gets just the first tupiles of the topics list
        # as a list itself to be fed to subscribe
        tqos = list(list(zip(*topics))[0])
        self._client.subscribe(tqos)

    def _on_message(self, client, userdata, msg):
        """
        called when the client receives a message that no
        specific topic callback handles
        """
        self.feedback_messages.append("Got unhandled message:{} = {}".format(msg.topic, msg.payload))


    def _handle_test(self, c, u, msg):
        self.feedback_messages.append("Got test msg")

    ########################
    # WARA-PS related stuff
    ########################
    def _tick_waraps(self):
        self._send_waraps_sensor()
        self._send_waraps_sensorinfo()
        self._send_waraps_hearbeat()

    def _send_waraps(self, json_dict, topic):
        """
        A simple way to send general stuff related to waraps
        """
        msgstr = json.dumps(json_dict, sort_keys=True, indent=4, separators=(',',': '))
        # XXX change the prefix to be dynamic
        # first one is one of: ground, air, surface, subsurface
        # second one is one of: real, simulation
        prefix = "waraps/unit/{}/{}".format(self._waraps_agent_type, "simulation")
        self._client.publish("{}/{}/{}".format(prefix, self.robot_name, topic),
                             msgstr,
                             qos = 0,
                             retain = False)

    def _send_waraps_sensor(self):
        j = {}
        # probably negatives for depth?
        j["altitude"] = -self._vehicle.depth
        j["latitude"] = self._vehicle.position_latlon[0]
        j["longitude"] = self._vehicle.position_latlon[1]
        j["rostype"] = "GeoPoint"
        self._send_waraps(j, 'sensor/position')

    def _send_waraps_sensorinfo(self):
        j = {}
        j['name'] = self.robot_name
        j['rate'] = common_globals.BT_TICK_RATE
        j['stamp'] = time.time()
        j['sensor_data_provided'] = ["position"]
        j['rostype'] = "SensorInfo"
        self._send_waraps(j, 'sensor_info')

    def _send_waraps_hearbeat(self):
        j = {}
        j['name'] = self.robot_name
        j['rate'] = common_globals.BT_TICK_RATE
        j['stamp'] = time.time()
        # XXX just a passive sensor until we know how to handle
        # wara-ps related commands and stuff
        j['levels'] = ["sensor"]
        j['agent-uuid'] = self._uuid
        j['agent-type'] = self._waraps_agent_type
        j['agent-description'] = "A SMaRC test robot"
        j['rostype'] = "HeartBeat"
        self._send_waraps(j, 'heartbeat')




if __name__ == '__main__':
    h = MQTTHandler()

    while True:
        print('='*10)
        for msg in h.feedback_messages:
            print(msg)

        h.tick()
        time.sleep(1./common_globals.BT_TICK_RATE)




