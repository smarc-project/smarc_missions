#include "sam_action_servers/emergency_server.hpp"

EmergencyServer::EmergencyServer(ros::NodeHandle nh) : nh_(&nh)
{
    std::string thruster_1_top, thruster_2_top, vbs_cmd_top;
    nh_->param<std::string>(("thruster_1_top"), thruster_1_top, "core/thruster_1_cmd");
    nh_->param<std::string>(("thruster_2_top"), thruster_2_top, "core/thruster_2_cmd");
    nh_->param<std::string>(("vbs_cmd_top"), vbs_cmd_top, "vbs_cmd_top");
    thrust_1_cmd_pub_ = nh_->advertise<smarc_msgs::ThrusterRPM>(thruster_1_top, 1);
    thrust_2_cmd_pub_ = nh_->advertise<smarc_msgs::ThrusterRPM>(thruster_2_top, 1);
    vbs_cmd_pub_ = nh_->advertise<sam_msgs::PercentStamped>(vbs_cmd_top, 1);

    std::string emergency_as;
    nh_->param<std::string>(("emergency_as"), emergency_as, "emergency_as");
    as_ = new actionlib::SimpleActionServer<smarc_bt::GotoWaypointAction>(*nh_, emergency_as,
                                                                          boost::bind(&EmergencyServer::emergencyCB, this, _1),
                                                                          false);
    as_->start();
}

void EmergencyServer::emergencyCB(const smarc_bt::GotoWaypointGoalConstPtr &goal)
{
    this->handle_emergency();
}

void EmergencyServer::handle_emergency()
{
    ROS_ERROR("-------------------------------------------------------------------");
    ROS_ERROR_STREAM("Emergency manager: "
                        << "Emergency CB received. Bringing vehicle to surface");
    ROS_ERROR("-------------------------------------------------------------------");

    ros::Time start_t = ros::Time::now();
    ros::Duration backup_duration(1.0);

    smarc_msgs::ThrusterRPM rpm_msg;
    sam_msgs::PercentStamped vbs_msg;
    vbs_msg.value = 0.;
    ros::Rate r(10.);

    // The only way out of this loop is preemption or killing the node
    while (!as_->isPreemptRequested())
    {
        if (ros::Time::now() < start_t + backup_duration)
        {
            rpm_msg.rpm = -500;
            thrust_1_cmd_pub_.publish(rpm_msg);
            thrust_2_cmd_pub_.publish(rpm_msg);
        }
        else
        {
            // TODO: disable thrusters controllers here
            rpm_msg.rpm = 0;
            thrust_1_cmd_pub_.publish(rpm_msg);
            thrust_2_cmd_pub_.publish(rpm_msg);
        }
        // TODO: disable VBS controller here
        vbs_msg.header.stamp = ros::Time::now();
        vbs_cmd_pub_.publish(vbs_msg);
        r.sleep();
    }
    ROS_INFO_STREAM("Emergency manager: emergency preempted ");
    // set the action state to preempted
    as_->setPreempted();
}
