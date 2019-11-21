
# Copyright (c) 2014 STRANDS

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import roslib
import rospy
import web
import os

import strands_webserver.client_utils
import strands_webserver.page_utils

from smarc_planning_msgs.msg import ConditionalAction, ExecutionStatus

#from strands_admin_web_ui.services import TaskDemander
#import strands_admin_web_ui.page_utils

template_dir = roslib.packages.get_pkg_dir('auv_sm_mission_executor') + '/templates'
render = web.template.render(template_dir)
www_prefix = roslib.packages.get_pkg_dir('auv_sm_mission_executor') + '/www/'

def generate_interface_page(file_name, left = "", right = "", script = ""):
    """ Create a page by rendering the strands_admin_web_ui/templates/index.html using web.py and place it at strands_admin_web_ui/www/ """

    full_path = www_prefix + file_name
    rospy.loginfo('generate page %s' % full_path)

    try:
        os.makedirs(os.path.dirname(full_path))
    except OSError:
        pass

    page = str(render.index(left, right, script))
    with open(full_path, 'w+') as f:
        f.write(page)


def get_schedule_display(element="#left_div"):
    """ Generate the javascript to be which displays a schedule display in the named div element. """
    return str(render.schedule_display(element))

#def get_map_display(element="bottom_left"):
#    """ Generate the javascript to be which displays a schedule display in the named div element. """
#    return str(render.map(element))

#def get_video_display(element="bottom_right"):
#    """ Generate the javascript to be which displays a schedule display in the named div element. """
#    return str(render.mjpeg_stream(element))

#def get_task_event_display(element="#left_div"):
#    """ Generate the javascript to be which displays task event details  in the named div element. """
#    return str(render.task_event_display(element))

def get_service_button(button_text, service, element="#bottom_right_div"):
    """ Generate a button that calls the std_srvs/Empty service when pressed """
    print "Adding a service button!"
    return str(render.service_button(button_text, service, element))

#def generate_buttons_for_tasks(tasks_and_labels):
#    demander = TaskDemander()
#
#    output = ""
#
#    for task, label in tasks_and_labels:
#        demand_srv = demander.offer_demand_service(task)
#        output += strands_admin_web_ui.page_utils.get_service_button(label, demand_srv)
#
#    return output, demander
