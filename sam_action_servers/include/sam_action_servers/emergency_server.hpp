#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <smarc_bt/GotoWaypointAction.h>
#include <smarc_msgs/ThrusterRPM.h>
#include <sam_msgs/PercentStamped.h>

class EmergencyServer
{

public:
    ros::NodeHandle *nh_;
    actionlib::SimpleActionServer<smarc_bt::GotoWaypointAction> *as_;
    smarc_bt::GotoWaypointFeedback as_fb_;
    smarc_bt::GotoWaypointResult as_result_;
    ros::Publisher thrust_1_cmd_pub_, thrust_2_cmd_pub_, vbs_cmd_pub_;

    EmergencyServer(ros::NodeHandle nh);

    void emergencyCB(const smarc_bt::GotoWaypointGoalConstPtr &goal);

    void handle_emergency();
};