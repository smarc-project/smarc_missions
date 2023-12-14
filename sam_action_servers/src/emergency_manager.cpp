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

    void emergencyCB(const smarc_bt::GotoWaypointGoalConstPtr &goal)
    {
        this->handle_emergency();
    }

    void handle_emergency()
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
};

class GenericSensorMonitor
{

public:

    std::string topic_name_, topic_end_;
    double rate_;
    ros::CallbackQueue queue;
    ros::Subscriber subscriber_;
    ros::Publisher abort_pub_;
    actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> *ac_g2wp_;
    actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> *ac_emerg_;
    ros::NodeHandle *nh_;
    bool topic_up_, subs_init_;
    ros::master::V_TopicInfo master_topics_;

    GenericSensorMonitor(std::string &topic, double rate, actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> &ac_g2wp,
                         actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction>& ac_emerg, ros::Publisher& abort_pub) : 
                         topic_name_(topic), rate_(rate), ac_g2wp_(&ac_g2wp), ac_emerg_(&ac_emerg), abort_pub_(abort_pub)
    {
        // Each monitor has its own nh and queue
        nh_ = new ros::NodeHandle("~");
        nh_->setCallbackQueue(&queue);
        
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

    void MonitorTopic()
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
                        ROS_INFO_STREAM("Sensor monitor: listening to " << topic_name_);
                        subscriber_ = nh_->subscribe(topic_name_, 10, &GenericSensorMonitor::topicCB, this);
                        subs_init_ = true;
                    }
                }
            }

            // If msg in queue
            if(!queue.isEmpty())
            {
                queue.callOne(ros::WallDuration(0.01));
                topic_up_ = true;
            }
            // If the queue is empty: 
            else
            {
                // If the queue is empty but it shouldn't be, emergency
                if(topic_up_)
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

    void emergency_detected()
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
    void topicCB(const ShapeShifter::ConstPtr& msg)
    {
    }

};


void handle_abort_leak(const ShapeShifter::ConstPtr &msg,
                       actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> &ac_g2wp,
                       actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> &ac_emerg,
                       bool& emergency_trigger)
{
    if(!emergency_trigger)
    {
        const std::string &datatype = msg->getDataType();
        emergency_trigger = true;
        ROS_ERROR_STREAM("Emergency manager: "
                        << datatype << " received, starting emergency surfacing");
        // Cancel current goal of WP follower
        ac_g2wp.cancelAllGoals();

        // Request emergency surface action
        smarc_bt::GotoWaypointGoal goal;
        ac_emerg.sendGoal(goal);
    }
}

// Monitors that all relevant topics are up and running and the state of the ROS master
void vehicle_state_cb(std::vector<std::shared_ptr<GenericSensorMonitor>> &monitors,
                      ros::Publisher &vehicle_state_pub, std::shared_ptr<EmergencyServer> emergency_server)
{
    if (!ros::master::check())
    {
        ROS_ERROR("-------------------------------------------------------------------");
        ROS_ERROR("Sensor monitor: ROS master is down, triggering emergency surfacing manually");
        ROS_ERROR("-------------------------------------------------------------------");
        // If the master is down, the action server will no respond. Carry out emergency surfacing manually
        // There's no point here on trying to preempt the current WP action. It will not be transmitted
        emergency_server->handle_emergency();
    }

    std_msgs::Bool vehicle_ready;
    vehicle_ready.data = true;
    for (std::shared_ptr<GenericSensorMonitor> monitor : monitors)
    {
        if(!monitor->topic_up_)
        {
            vehicle_ready.data = false;
            break;
        }
    }
    if(vehicle_ready.data)
    {
        vehicle_state_pub.publish(vehicle_ready);
    }
}


int main(int argn, char* args[])
{
    ros::init(argn, args, "emergency_manager");
    ros::NodeHandle nh("~");
    
    // Emergency surface action server
    std::shared_ptr<EmergencyServer> emergency_server(new EmergencyServer(nh));

    // Basic topics monitors: check that data is coming in on the defined topics. Signals an emergency when it stops
    std::string emerg_config, vehicle_ready_top, goto_wp_action, emergency_as;
    double vehicle_ready_period;
    nh.param<std::string>(("emergency_config_file"), emerg_config, "/home/torroba/catkin_workspaces/smarc_ws/src/smarc_missions/sam_action_servers/config/emergency_config.yaml");
    boost::filesystem::path emerg_config_path(emerg_config);
    if (!boost::filesystem::exists(emerg_config_path))
    {
        ROS_ERROR_STREAM("Emergency manager config file doesn't exist " << emerg_config_path.string());
        exit(0);
    }

    nh.param<std::string>(("goto_wp_action"), goto_wp_action, "goto_wp_action");
    actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction>  ac_g2wp(goto_wp_action, true);
    nh.param<std::string>(("emergency_as"), emergency_as, "emergency_as");
    actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> ac_emerg(emergency_as, true);
    while (!ac_g2wp.waitForServer() && !ac_emerg.waitForServer() && ros::ok())
    {
        ROS_WARN("Waiting for servers ");
        ros::Duration(1.).sleep();
    }
    ROS_INFO("Emergency node: action servers instantiated");

    std::string abort_top;
    nh.param<std::string>(("abort_top"), abort_top, "/sam/core/abort");
    ros::Publisher abort_pub = nh.advertise<std_msgs::Empty>(abort_top, 10);

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
            monitors.emplace_back(new GenericSensorMonitor(topic, freq * 0.9, ac_g2wp, ac_emerg, abort_pub)); // Decrease slightly freq to account for slow transmission at times
        }
    }

    // Monitor state of the vehicle to let the BT know when it's ready for a mission
    nh.param<std::string>(("vehicle_ready_top"), vehicle_ready_top, "vehicle_ready");
    nh.param<double>(("vehicle_ready_period"), vehicle_ready_period, 1.);
    ros::Publisher vehicle_state_pub = nh.advertise<std_msgs::Bool>(vehicle_ready_top, 1);
    boost::function<void (const ros::TimerEvent&)> timer_cb = [&monitors, &vehicle_state_pub, &emergency_server]
    (const ros::TimerEvent &)
    {
        vehicle_state_cb(monitors, vehicle_state_pub, emergency_server);
    };
    ros::Timer vehicle_state_timer = nh.createTimer(ros::Duration(vehicle_ready_period), timer_cb);

    // Handle leaks or abort messages, equally and only once
    bool emergency_trigger = false;
    boost::function<void(const ShapeShifter::ConstPtr &)> abort_leak_cb = [&ac_g2wp, &ac_emerg, &emergency_trigger]
    (const ShapeShifter::ConstPtr &msg)
    {
        handle_abort_leak(msg, ac_g2wp, ac_emerg, emergency_trigger);
    };

    std::string leak_top;
    nh.param<std::string>(("leak_top"), leak_top, "leak");
    ros::Subscriber leak_subs = nh.subscribe(leak_top, 1, abort_leak_cb);
    ros::Subscriber abort_subs = nh.subscribe(abort_top, 1, abort_leak_cb);

    ros::spin();

    return 0;
}
