#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include "ros_type_introspection/ros_introspection.hpp"
#include <topic_tools/shape_shifter.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <smarc_bt/GotoWaypointAction.h>
#include <smarc_msgs/ThrusterRPM.h>
#include <sam_msgs/PercentStamped.h>
#include <smarc_msgs/Leak.h>

#include <thread>

using namespace RosIntrospection;
using topic_tools::ShapeShifter;

class GenericSensorMonitor
{

public:
    std::string topic_name_, topic_end_;
    double rate_;
    ros::CallbackQueue queue_;
    ros::Subscriber subscriber_;
    ros::Publisher abort_pub_;
    actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> *ac_g2wp_;
    actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> *ac_emerg_;
    ros::NodeHandle *nh_;
    bool topic_up_, subs_init_;
    ros::master::V_TopicInfo master_topics_;

    GenericSensorMonitor(std::string &topic, double rate, actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> &ac_g2wp,
                         actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> &ac_emerg, ros::Publisher &abort_pub);

    void MonitorTopic();
    void emergency_detected();
    // Callbacks
    void topicCB(const ShapeShifter::ConstPtr &msg);
};