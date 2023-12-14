#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <std_msgs/Bool.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TwistStamped.h"
#include <std_msgs/Empty.h>
#include "ros_type_introspection/ros_introspection.hpp"
#include <topic_tools/shape_shifter.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <smarc_bt/GotoWaypointAction.h>
#include <smarc_msgs/ThrusterRPM.h>
#include <sam_msgs/PercentStamped.h>

#include <thread>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

using namespace RosIntrospection;
using topic_tools::ShapeShifter;

class EmergencyServer
{

public:

    ros::NodeHandle* nh_;
    actionlib::SimpleActionServer<smarc_bt::GotoWaypointAction>* as_;
    smarc_bt::GotoWaypointFeedback as_fb_;
    smarc_bt::GotoWaypointResult as_result_;
    ros::Publisher thrust_1_cmd_pub_, thrust_2_cmd_pub_, vbs_cmd_pub_;

    EmergencyServer(ros::NodeHandle nh): nh_(&nh)
    {
        std::string thruster_1_top, thruster_2_top, vbs_cmd_top;
        nh_->param<std::string>(("thruster_1_top"), thruster_1_top, "core/thruster_1_cmd");
        nh_->param<std::string>(("thruster_2_top"), thruster_2_top, "core/thruster_2_cmd");
        nh_->param<std::string>(("vbs_cmd_top"), vbs_cmd_top, "core/thruster_2_cmd");
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

    void emergencyCB(const smarc_bt::GotoWaypointGoalConstPtr &goal)
    {
        ROS_ERROR_STREAM_NAMED("Emergency manager ", "Emergency CB received");
        ros::Time start_t = ros::Time::now();
        ros::Duration backup_duration(3.0);

        smarc_msgs::ThrusterRPM rpm_msg;
        sam_msgs::PercentStamped vbs_msg;
        vbs_msg.value = 0.;
        ros::Rate r(10.);

        while(!as_->isPreemptRequested() && ros::ok())
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
        ROS_INFO_STREAM("Emergency preempted ");
        // set the action state to preempted
        as_->setPreempted();
    }

};

class GenericSensorMonitor
{

public:

    std::string topic_name_, topic_end_;
    double rate_;
    ros::CallbackQueue queue;
    ros::Subscriber subscriber_;
    ros::Publisher abort_pub_;
    actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> *ac_;
    ros::NodeHandle *nh_;
    ros::Publisher thrust_1_cmd_pub_, thrust_2_cmd_pub_, vbs_cmd_pub_;
    bool queue_init_, subs_init_;
    ros::master::V_TopicInfo master_topics_;

    GenericSensorMonitor(std::string &topic, double rate) : topic_name_(topic), rate_(rate)
    {
        nh_ = new ros::NodeHandle("~");
        nh_->setCallbackQueue(&queue);
        
        std::string abort_top, emergency_as;
        nh_->param<std::string>(("abort_top"), abort_top, "/sam/core/abort");
        abort_pub_ = nh_->advertise<std_msgs::Empty>(abort_top, 10);

        std::string thruster_1_top, thruster_2_top, vbs_cmd_top;
        nh_->param<std::string>(("thruster_1_top"), thruster_1_top, "core/thruster_1_cmd");
        nh_->param<std::string>(("thruster_2_top"), thruster_2_top, "core/thruster_2_cmd");
        nh_->param<std::string>(("vbs_cmd_top"), vbs_cmd_top, "core/thruster_2_cmd");
        thrust_1_cmd_pub_ = nh_->advertise<smarc_msgs::ThrusterRPM>(thruster_1_top, 1);
        thrust_2_cmd_pub_ = nh_->advertise<smarc_msgs::ThrusterRPM>(thruster_2_top, 1);
        vbs_cmd_pub_ = nh_->advertise<sam_msgs::PercentStamped>(vbs_cmd_top, 1);

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
        queue_init_ = false;
        subs_init_ = false;
        std::thread(&GenericSensorMonitor::CheckQueue, this).detach();
    }

    void CheckQueue()
    {
        ros::Rate r(rate_);
        while(ros::ok())
        {
            // Check if topic exists, then subscribe 
            if(!subs_init_)
            {
                ros::master::getTopics(master_topics_);

                for (ros::master::V_TopicInfo::iterator it = master_topics_.begin(); it != master_topics_.end(); it++)
                {
                    const ros::master::TopicInfo &info = *it;
                    if(topic_name_ == info.name)
                    {
                        ROS_INFO_STREAM_NAMED("Emergency manager: ", "listening to " << topic_name_);
                        subscriber_ = nh_->subscribe(topic_name_, 10, &GenericSensorMonitor::topicCB, this);
                        subs_init_ = true;
                    }
                }
            }

            // If msg in queue
            if(!queue.isEmpty())
            {
                queue.callOne(ros::WallDuration(0.01));
                queue_init_ = true;
            }
            // If the queue is empty: 
            else
            {
                // If the queue is empty but it shouldn't be, emergency
                if(queue_init_)
                {
                    ROS_ERROR_STREAM_NAMED("Emergency manager: ", "data not coming in " << topic_name_ + " aborting mission");
                    std_msgs::Empty abort;
                    abort_pub_.publish(abort);
                    if (topic_end_ == "heartbeat")
                    {
                        ROS_ERROR_STREAM_NAMED("Emergency manager ", "BT is down, emergency action triggered manually");
                        this->emergency_no_bt();
                    }
                    break;
                }
                // If the queue is empty because the data flow has not started yet, throw warning
                else
                {
                    ROS_WARN_STREAM_THROTTLE_NAMED(int(rate_), "Emergency manager: ", "data stream not initialized " << topic_name_);
                }
            }
            
            r.sleep();
        }    
    }

    void emergency_no_bt()
    {
        ros::Time start_t = ros::Time::now();
        ros::Duration backup_duration(1.0);

        smarc_msgs::ThrusterRPM rpm_msg;
        sam_msgs::PercentStamped vbs_msg;
        vbs_msg.value = 0.;
        ros::Rate r(10.);

        while (ros::ok())
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
    }

    // Callbacks
    void topicCB(const ShapeShifter::ConstPtr& msg)
    {
    }

};


int main(int argn, char* args[])
{
    ros::init(argn, args, "emergency_management");
    ros::NodeHandle nh("~");
    
    // Emergency surface action server
    std::shared_ptr<EmergencyServer> emergency_server(new EmergencyServer(nh));

    // Basic topics monitors
    std::string emerg_config;
    nh.param<std::string>(("emergency_config_file"), emerg_config, "emergency_config.yaml");
    boost::filesystem::path emerg_config_path(emerg_config);

    if (!boost::filesystem::exists(emerg_config_path))
    {
        ROS_ERROR_STREAM_NAMED("Emergency manager ", "config file doesn't exit " << emerg_config_path.string());
        exit(0);
    }
    
    YAML::Node config = YAML::LoadFile(emerg_config_path.string());
    std::string robot_name = config["namespace"].as<std::string>();
    const YAML::Node &monitor = config["monitor"];
    std::vector<std::shared_ptr<GenericSensorMonitor>> monitors;
    for (YAML::const_iterator it = monitor.begin(); it != monitor.end(); ++it)
    {
        YAML::Node value = it->begin()->second;
        if (value.Type() == YAML::NodeType::Map)
        {
            std::string topic_name = value["name"].as<std::string>();
            double freq = value["freq"].as<double>();
            std::string topic = "/" + robot_name + "/" + topic_name;
            ros::Duration(0.01).sleep();
            monitors.emplace_back(new GenericSensorMonitor(topic, freq * 0.9)); // Decrease slightly freq to account for slow transmission at times
        }
    }

    ros::spin();

    return 0;
}