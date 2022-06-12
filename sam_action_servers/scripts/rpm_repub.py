#! /usr/bin/env python

#Wrapper node to read control inputs and republish at above 10hz 

import rospy
from smarc_msgs.msg import ThrusterRPM
from std_msgs.msg import Bool

class RPMRepub(object):

    def rpm1_cb(self,rpm):
        self.rpm1.rpm= rpm.rpm

    def rpm2_cb(self,rpm):
        self.rpm2.rpm= rpm.rpm

    # Callback function to check for enable flag
    def enable_cb(self,enable_msg):
        #print('Enable:', enable_msg.data)
        if (not enable_msg.data):
            self.enable_flag = False 
            rospy.loginfo_throttle(5,'rpm ctrl disabled')

        else:
            self.enable_flag = True
            rospy.loginfo_throttle(5,'rpm ctrl enabled')

    def __init__(self, name):
        as_rpm1_topic = rospy.get_param('~as_rpm1_topic', '/sam/ctrl/goto_waypoint/rpm1')
        as_rpm2_topic = rospy.get_param('~as_rpm2_topic', '/sam/ctrl/goto_waypoint/rpm2')
        rpm1_cmd_topic= rospy.get_param('~rpm1_cmd_topic', '/sam/core/thruster1_cmd')
        rpm2_cmd_topic = rospy.get_param('~rpm2_cmd_topic', '/sam/core/thruster2_cmd')
        enable_topic = rospy.get_param('~rpm_enable_topic', '/sam/ctrl/goto_waypoint/rpm/enable')


        self.loop_freq = rospy.get_param("~loop_freq", 21)

        self.rpm1_sub = rospy.Subscriber(as_rpm1_topic, ThrusterRPM, self.rpm1_cb)
        self.rpm2_sub = rospy.Subscriber(as_rpm2_topic, ThrusterRPM, self.rpm2_cb)

        rospy.Subscriber(enable_topic, Bool, self.enable_cb)

        self.rpm1_pub = rospy.Publisher(rpm1_cmd_topic, ThrusterRPM , queue_size=10)
        self.rpm2_pub = rospy.Publisher(rpm2_cmd_topic, ThrusterRPM , queue_size=10)
        

        self.rate = rospy.Rate(self.loop_freq) 

        #initialize actuator commands
        self.rpm1 = ThrusterRPM()
        self.rpm2 = ThrusterRPM()
        
        self.enable_flag = False

        while not rospy.is_shutdown():

            if self.enable_flag:
            
                #publish to actuators
                self.rpm1_pub.publish(self.rpm1)
                self.rpm2_pub.publish(self.rpm2)
                
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rpm_repub")
    rpm_repub_obj = RPMRepub(rospy.get_name())

