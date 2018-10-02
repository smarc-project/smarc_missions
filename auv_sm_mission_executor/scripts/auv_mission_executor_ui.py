#! /usr/bin/env python

import roslib
import rospy

import auv_sm_mission_executor.page_utils

from smarc_msgs.msg import SMTask, ExecutionStatus # TaskEvent
import strands_webserver.client_utils
import strands_webserver.page_utils
from std_srvs.srv import Empty

if __name__ == '__main__':
    rospy.init_node("auv_mission_executor_ui")

    # display some content
    display_no = rospy.get_param("~display", 0)

    if display_no == 0:
        rospy.loginfo('writing to all displays)')
    else:
        rospy.loginfo('writing to display: %s', display_no)

    prefix='auv_mission_executor_ui_server'
    
    # Setup -- must be done before other strands_admin_web_ui calls
    # serves pages relative to strands_admin_web_ui/www -- this is important as there as some javascript files there
    strands_webserver.client_utils.set_http_root(roslib.packages.get_pkg_dir('auv_sm_mission_executor') + '/www', prefix=prefix)
    rospy.loginfo('root set')

    page = 'index.html'

    right_div = '<div id="top_right_div"></div><div id="bottom_right_div"><div id="button_left_div" style="width:300px; float:left;"></div><div id="button_right_div" style="width:300px; float:left;"></div></div>'
    script = auv_sm_mission_executor.page_utils.get_schedule_display("#top_right_div")
    script += auv_sm_mission_executor.page_utils.get_service_button("Cancel active task", "sm_reset", "#button_left_div")
    script += auv_sm_mission_executor.page_utils.get_service_button("Cancel all tasks", "cancel_all_tasks", "#button_right_div")

    auv_sm_mission_executor.page_utils.generate_interface_page(page, right=right_div, script=script)
    rospy.loginfo('page generated')

    strands_webserver.client_utils.display_relative_page(display_no, page, prefix=prefix)
    rospy.spin()
