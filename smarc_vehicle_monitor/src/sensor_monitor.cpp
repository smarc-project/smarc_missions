#include "smarc_vehicle_monitor/sensor_monitor.hpp"

using namespace RosIntrospection;
using topic_tools::ShapeShifter;


GenericSensorMonitor::GenericSensorMonitor(std::string &topic, double rate, actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> &ac_g2wp,
                                         actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> &ac_emerg, ros::Publisher &abort_pub) : topic_name_(topic), rate_(rate), ac_g2wp_(&ac_g2wp), ac_emerg_(&ac_emerg), abort_pub_(abort_pub)
{
    // Each monitor has its own nh and queue
    nh_ = new ros::NodeHandle("~");
    nh_->setCallbackQueue(&queue_);

    // Get last word of the topic name. It'll be used to find the heartbeat one
    std::string delimiter = "/";
    size_t pos = 0;
    std::string token;
    topic_end_ = topic_name_;
    while ((pos = topic_end_.find(delimiter)) != std::string::npos)
    {
        token = topic_end_.substr(0, pos);
        topic_end_.erase(0, pos + delimiter.length());
    }

    // Flags to monitor topics
    topic_up_ = false;
    subs_init_ = false;
    std::thread(&GenericSensorMonitor::MonitorTopic, this).detach();
}

void GenericSensorMonitor::MonitorTopic()
{
    ros::Rate r(rate_);
    while (ros::ok())
    {
        // Check if topic exists, then subscribe
        if (!subs_init_)
        {
            ros::master::getTopics(master_topics_);
            for (ros::master::V_TopicInfo::iterator it = master_topics_.begin(); it != master_topics_.end(); it++)
            {
                const ros::master::TopicInfo &info = *it;
                if (topic_name_ == info.name)
                {
                    ROS_INFO_STREAM("Sensor monitor: listening to " << topic_name_);
                    subscriber_ = nh_->subscribe(topic_name_, 10, &GenericSensorMonitor::topicCB, this);
                    subs_init_ = true;
                }
            }
        }

        // If msg in queue
        if (!queue_.isEmpty())
        {
            queue_.callOne(ros::WallDuration(0.01));
            topic_up_ = true;
        }
        // If the queue is empty:
        else
        {
            // If the queue is empty but it shouldn't be, emergency
            if (topic_up_)
            {
                this->emergency_detected();
            }
            // If the queue is empty because the data flow has not started yet, throw warning
            else
            {
                ROS_WARN_STREAM_THROTTLE(int(rate_), "Sensor monitor: data stream not initialized " << topic_name_);
            }
        }

        r.sleep();
    }
}

void GenericSensorMonitor::emergency_detected()
{
    ROS_ERROR("-------------------------------------------------------------------");
    ROS_ERROR_STREAM("Sensor monitor: data not coming in " << topic_name_ + " aborting mission");
    ROS_ERROR("-------------------------------------------------------------------");
    std_msgs::Empty abort;
    abort_pub_.publish(abort);
    topic_up_ = false; // And reset monitor

    // // If the BT itself is down, handle the emergency yourself
    // if (topic_end_ == "heartbeat")
    // {
    //     ROS_ERROR_STREAM("Sensor monitor: "
    //                      << "BT is down, triggering emergency surfacing manually");

    //     // Cancel current goal of WP follower
    //     ac_g2wp_->cancelAllGoals();

    //     // Request emergency surface action
    //     smarc_bt::GotoWaypointGoal goal;
    //     ac_emerg_->sendGoalAndWait(goal);
    // }
}

// Callbacks
void GenericSensorMonitor::topicCB(const ShapeShifter::ConstPtr &msg)
{
}