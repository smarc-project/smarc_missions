#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import imc_enums, bb_enums
from imc_ros_bridge.msg import EstimatedState, VehicleState, PlanDB, PlanDBInformation, PlanDBState, PlanControlState, PlanControl, PlanSpecification, Maneuver
from sensor_msgs.msg import NavSatFix

import rospy, time
import numpy as np

from mission_plan import MissionPlan, Waypoint

class NeptusHandler(object):
    """
    A class to keep everthing related to Neptus and IMC
    in one place.
    """
    def __init__(self,
                 auv_config,
                 vehicle,
                 blackboard):

        self._vehicle = vehicle
        self._config = auv_config
        self._bb = blackboard

        self._estimated_state_msg = EstimatedState()
        self._estimated_state_pub = rospy.Publisher(self._config.ESTIMATED_STATE_TOPIC,
                                                    EstimatedState,
                                                    queue_size=1)

        self._gps_fix_msg = NavSatFix()
        self._gps_fix_pub = rospy.Publisher(self._config.GPSFIX_TOPIC,
                                            NavSatFix,
                                            queue_size=1)
        self._gps_nav_data_pub = rospy.Publisher(self._config.GPS_NAV_DATA_TOPIC,
                                                 NavSatFix,
                                                 queue_size=1)

        self._plan_control_state_msg = PlanControlState()
        self._plan_control_state_pub = rospy.Publisher(self._config.PLAN_CONTROL_STATE_TOPIC,
                                                       PlanControlState,
                                                       queue_size=1)

        self._vehicle_state_msg = VehicleState()
        self._vehicle_state_pub = rospy.Publisher(self._config.VEHICLE_STATE_TOPIC,
                                                  VehicleState,
                                                  queue_size=1)

        # plandb has multiple messages that are easier to create on demand
        self._last_received_plandb_msg = None
        self._plandb_pub = rospy.Publisher(self._config.PLANDB_TOPIC,
                                           PlanDB,
                                           queue_size=1)
        self._plandb_sub = rospy.Subscriber(self._config.PLANDB_TOPIC,
                                            PlanDB,
                                            callback=self._plandb_cb,
                                            queue_size=1)

        self._last_received_plancontrol_msg= None
        self._plancontrol_sub = rospy.Subscriber(self._config.PLAN_CONTROL_TOPIC,
                                                 PlanControl,
                                                 self._plancontrol_cb)

        # a list of messages from all the different parts of the handler
        self.feedback_messages = []


    def __str__(self):
        s = "Neptus status:\n"
        for m in self.feedback_messages:
            s += "\n"+m
        return s

    def tick(self):
        self._updateEstimatedState()
        self._updatePlanControlState()
        self._updateVehicleState()
        self._updatePlanDB()
        self._updatePlanControl()
        self._updateGPSFix()
        self.feedback_messages = []

    def _updateEstimatedState(self):
        lat, lon = self._vehicle.position_latlon
        depth = self._vehicle.depth
        roll, pitch, yaw = self._vehicle.orientation_rpy

        if depth is None:
            depth = 0 # neptus _requires_ a depth

        if None in [lat, lon, yaw]:
            self.feedback_messages.append("EstimatedState could not be updated, lat,lon,yaw is None")
            return

        self._estimated_state_msg.lat = np.radians(lat)
        self._estimated_state_msg.lon= np.radians(lon)
        self._estimated_state_msg.depth = depth
        self._estimated_state_msg.psi = np.pi/2. - yaw
        # send the message to neptus
        self._estimated_state_pub.publish(self._estimated_state_msg)

    def _updateGPSFix(self):
        # the bridge only looks at lat lon height=altitude
        # we read this from the raw gps instead
        ros_gps_msg = self._vehicle.raw_gps_obj
        if ros_gps_msg is not None:
            self._gps_fix_msg.latitude = ros_gps_msg.latitude
            self._gps_fix_msg.longitude = ros_gps_msg.longitude
            self._gps_fix_msg.altitude = -self._vehicle.depth
            self._gps_fix_msg.header.seq = int(time.time())
            self._gps_fix_pub.publish(self._gps_fix_msg)
            self._gps_nav_data_pub.publish(self._gps_fix_msg)

    def _updatePlanControlState(self):
        tip_name = self._bb.get(bb_enums.TREE_TIP_NAME)
        tip_status = self._bb.get(bb_enums.TREE_TIP_STATUS)

        # this tip_status looks like: "Status.FAILURE"
        # I just wanna get the first letter after dot.
        self._plan_control_state_msg.man_id = tip_name+'('+tip_status[7]+')'

        mission_plan = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None:
            self._plan_control_state_msg.plan_id = 'No plan'
            self._plan_control_state_msg.plan_progress = 100.0
        elif mission_plan.is_complete():
            self._plan_control_state_msg.plan_id = 'Mission complete'
            self._plan_control_state_msg.plan_progress = 100.0
        else:
            current_wp_index = mission_plan.current_wp_index
            current_man_id = mission_plan.waypoint_man_ids[current_wp_index]
            total = len(mission_plan.waypoints)
            self._plan_control_state_msg.plan_id = str(mission_plan.plan_id)
            if mission_plan.plan_is_go:
                self._plan_control_state_msg.man_id = current_man_id

            plan_progress = (current_wp_index * 100.0) / total # percent float
            self._plan_control_state_msg.plan_progress = plan_progress


        if tip_name in imc_enums.EXECUTING_ACTION_NAMES:
            self._plan_control_state_msg.state = imc_enums.STATE_EXECUTING
        elif tip_name in imc_enums.BLOCKED_ACTION_NAMES:
            self._plan_control_state_msg.state = imc_enums.STATE_BLOCKED
            self._plan_control_state_msg.plan_id = 'SAFETY FALLBACK'
            self._plan_control_state_msg.man_id = 'EMERGENCY'
            self._plan_control_state_msg.plan_progress = 0.0
        else:
            self._plan_control_state_msg.state = imc_enums.STATE_READY

        if self._bb.get(bb_enums.ENABLE_AUTONOMY):
            self._plan_control_state_msg.plan_id += '(AUTONOMOUS)'

        # send message to neptus
        self._plan_control_state_pub.publish(self._plan_control_state_msg)


    def _updateVehicleState(self):
        tip_name = self._bb.get(bb_enums.TREE_TIP_NAME)
        if tip_name in imc_enums.EXECUTING_ACTION_NAMES:
            self._vehicle_state_msg.op_mode = imc_enums.OP_MODE_MANEUVER
        elif tip_name == 'A_EmergencySurface':
            self._vehicle_state_msg.op_mode = imc_enums.OP_MODE_ERROR
        else:
            self._vehicle_state_msg.op_mode = imc_enums.OP_MODE_SERVICE

        self._vehicle_state_pub.publish(self._vehicle_state_msg)

    ##### PLANDB STUFF BEGINS HERE
    def _plandb_cb(self, msg):
        self._last_received_plandb_msg = msg

    def _make_plandb_info(self):
        current_mission_plan = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        plan_info = PlanDBInformation()
        plan_info.plan_id = current_mission_plan.plan_id
        plan_info.md5 = current_mission_plan.plandb_msg.plan_spec_md5
        plan_info.change_time = current_mission_plan.creation_time/1000.0
        return plan_info

    def _handle_request_get_info(self, plandb_msg):
        # we need to respond to this with some info... but what?
        rospy.loginfo_throttle_identical(30, "Got REQUEST GET_INFO planDB msg from Neptus")

        current_mission_plan = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        if current_mission_plan is None:
            return

        response = PlanDB()
        response.plan_id = current_mission_plan.plan_id
        response.type = imc_enums.PLANDB_TYPE_SUCCESS
        response.op = imc_enums.PLANDB_OP_GET_INFO
        response.plandb_information = self._make_plandb_info()
        self._plandb_pub.publish(response)
        rospy.loginfo_throttle_identical(30, "Answered GET_INFO for plan:"+str(response.plan_id))

    def _handle_request_get_state(self, plandb_msg):
        rospy.loginfo_throttle_identical(30, "Got REQUEST GET_STATE planDB msg from Neptus")
        current_mission_plan = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        if current_mission_plan is None:
            return

        # https://github.com/LSTS/imcjava/blob/d95fddeab4c439e603cf5e30a32979ad7ace5fbc/src/java/pt/lsts/imc/adapter/PlanDbManager.java#L160
        # See above for an example
        response = PlanDB()
        response.plan_id = current_mission_plan.plan_id
        response.type = imc_enums.PLANDB_TYPE_SUCCESS
        response.op = imc_enums.PLANDB_OP_GET_STATE

        response.plandb_state = PlanDBState()
        response.plandb_state.plan_count = 1
        response.plandb_state.plans_info.append(self._make_plandb_info())

        self._plandb_pub.publish(response)
        rospy.loginfo_throttle_identical(30, "Answered GET_STATE for plan:\n"+str(response.plan_id))

    def _handle_set_plan(self, plandb_msg):
        # there is a plan we can at least look at
        mission_plan = MissionPlan(plan_frame = self._config.UTM_LINK,
                                   plandb_msg = plandb_msg,
                                   latlontoutm_service_name = self._config.LATLONTOUTM_SERVICE,
                                   latlontoutm_service_name_alternative = self._config.LATLONTOUTM_SERVICE_ALTERNATIVE,
                                   coverage_swath = self._bb.get(bb_enums.SWATH),
                                   vehicle_localization_error_growth = self._bb.get(bb_enums.LOCALIZATION_ERROR_GROWTH))

        if mission_plan.no_service:
            self.feedback_messages.append("MISSION PLAN HAS NO SERVICE")
            return

        self._bb.set(bb_enums.MISSION_PLAN_OBJ, mission_plan)
        self._bb.set(bb_enums.ENABLE_AUTONOMY, False)
        self._bb.set(bb_enums.MISSION_FINALIZED, False)
        rospy.loginfo_throttle_identical(5, "Set the mission plan to:{} and un-finalized the mission.".format(mission_plan))

    def _handle_plandb_msg(self):
        plandb_msg = self._last_received_plandb_msg
        if plandb_msg is None:
            return

        typee = plandb_msg.type
        op = plandb_msg.op

        if typee == imc_enums.PLANDB_TYPE_REQUEST and op == imc_enums.PLANDB_OP_GET_INFO:
            self._handle_request_get_info(plandb_msg)

        elif typee == imc_enums.PLANDB_TYPE_REQUEST and op == imc_enums.PLANDB_OP_GET_STATE:
            self._handle_request_get_state(plandb_msg)

        elif typee == imc_enums.PLANDB_TYPE_SUCCESS and op == imc_enums.PLANDB_OP_SET:
            self.feedback_messages.append("Got SUCCESS for plandb set")

        elif typee == imc_enums.PLANDB_TYPE_SUCCESS and op == imc_enums.PLANDB_OP_GET_INFO:
            self.feedback_messages.append("Got SUCCESS for plandb get info")

        elif typee == imc_enums.PLANDB_TYPE_SUCCESS and op == imc_enums.PLANDB_OP_GET_STATE:
            self.feedback_messages.append("Got SUCCESS for plandb get state")

        elif op == imc_enums.PLANDB_OP_SET:
            self._handle_set_plan(plandb_msg)

        else:
            self.feedback_messages.append("Got some unhandled planDB message:\n"+str(plandb_msg))

    def _respond_set_success(self):
        current_mission_plan = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        if current_mission_plan is None:
            self.feedback_messages.append("No mission plan obj!")
            return

        plan_id = current_mission_plan.plan_id
        plandb_msg = PlanDB()
        plandb_msg.type = imc_enums.PLANDB_TYPE_SUCCESS
        plandb_msg.op = imc_enums.PLANDB_OP_SET
        plandb_msg.plan_id = plan_id
        self._plandb_pub.publish(plandb_msg)
        self.feedback_messages.append("Answered set success for plan_id:"+str(plan_id))

    def _updatePlanDB(self):
        self._respond_set_success()
        self._handle_plandb_msg()
        self._last_received_plandb_msg = None
    ###### PLANDB STUFF ENDS HERE

    def _plancontrol_cb(self, msg):
        self._last_received_plancontrol_msg = msg

    def _updatePlanControl(self):
        plan_control_msg = self._last_received_plancontrol_msg
        if plan_control_msg is None:
            # not receiving anything is ok.
            return

        # check if this message is a 'go' or 'no go' message
        # imc/plan_control(569):
        # int type:[0,1,2,3] req,suc,fail,in prog
        # int op:[0,1,2,3] start, stop, load, get
        # int request_id
        # string plan_id
        # int flags
        # string info

        # the start button in neptus sends:
        # type:0 op:0 plan_id:"string" flags:1
        # stop button sends:
        # type:0 op:1 plan_id:'' flags:1
        # teleop button sends:
        # type:0 op:0 plan_id:"teleoperation-mode" flags:0

        typee = plan_control_msg.type
        op = plan_control_msg.op
        plan_id = plan_control_msg.plan_id
        flags = plan_control_msg.flags

        # somehow this happens...
        if plan_id is None:
            plan_id=''

        current_mission_plan = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        # separate well-defined ifs for possible future shenanigans.
        if typee==0 and op==0 and plan_id!='' and flags==1:
            # start button
            # check if the start was given for our current plan
            current_mission_plan.plan_is_go = True
            self._bb.set(bb_enums.ENABLE_AUTONOMY, False)
            if current_mission_plan is not None and plan_id == current_mission_plan.plan_id:
                rospy.loginfo("Started plan:{}".format(plan_id))
            else:
                if current_mission_plan is None:
                    rospy.logwarn("Start given for plan:{} but we don't have a plan!".format(plan_id))
                else:
                    rospy.logwarn("Start given for plan:{} our plan:{}".format(plan_id, current_mission_plan.plan_id))

        if typee==0 and op==1 and plan_id=='' and flags==1:
            # stop button
            current_mission_plan.plan_is_go = False
            self._bb.set(bb_enums.ENABLE_AUTONOMY, False)

        # this string is hardcoded in Neptus, so we hardcode it here too!
        if typee==0 and op==0 and plan_id=='teleoperation-mode' and flags==0:
            # teleop button
            self._bb.set(bb_enums.ENABLE_AUTONOMY, True)
            rospy.logwarn_throttle_identical(10, "AUTONOMOUS MODE")

        # reset it until next message
        self._last_received_plancontrol_msg = None





