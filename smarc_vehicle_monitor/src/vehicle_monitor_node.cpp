#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <smarc_bt/GotoWaypointAction.h>
#include <smarc_msgs/Leak.h>

#include <thread>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

#include "smarc_vehicle_monitor/sensor_monitor.hpp"
#include "sam_action_servers/emergency_server.hpp"
#include "smarc_vehicle_monitor/node_monitor.hpp"

void handle_leak(const smarc_msgs::Leak::ConstPtr &msg,
                 actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> &ac_g2wp,
                 actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> &ac_emerg,
                 ros::Publisher &abort_pub,
                 bool &emergency_trigger)
{
    if (msg->value)
    {
        if (!emergency_trigger)
        {
            emergency_trigger = true;
            ROS_ERROR("-------------------------------------------------------------------");
            ROS_ERROR_STREAM("Emergency manager: "
                             << "leak msg received, starting emergency surfacing");
            ROS_ERROR("-------------------------------------------------------------------");
            // Cancel current goal of WP follower
            ac_g2wp.cancelAllGoals();
            // Request emergency surface action
            smarc_bt::GotoWaypointGoal goal;
            ac_emerg.sendGoal(goal);
            // Let the rest of the system now
            std_msgs::Empty abort;
            abort_pub.publish(abort);
        }
    }
}

void handle_abort(const std_msgs::Empty::ConstPtr &msg,
                  actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> &ac_g2wp,
                  actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> &ac_emerg,
                  bool &emergency_trigger)
{
    if (!emergency_trigger)
    {
        emergency_trigger = true;
        ROS_ERROR("-------------------------------------------------------------------");
        ROS_ERROR_STREAM("Emergency manager: "
                         << "abort msg received, starting emergency surfacing");
        ROS_ERROR("-------------------------------------------------------------------");
        // Cancel current goal of WP follower
        ac_g2wp.cancelAllGoals();
        // Request emergency surface action
        smarc_bt::GotoWaypointGoal goal;
        ac_emerg.sendGoal(goal);
    }
}


// Monitors that all relevant topics are up and running and the state of the ROS master
void vehicle_state_cb(std::vector<std::shared_ptr<GenericSensorMonitor>> &monitors,
                      ros::Publisher &vehicle_state_pub)
{
    std_msgs::Bool vehicle_ready;
    vehicle_ready.data = true;
    for (std::shared_ptr<GenericSensorMonitor> monitor : monitors)
    {
        if (!monitor->topic_up_)
        {
            vehicle_ready.data = false;
            break;
        }
    }
    if (vehicle_ready.data)
    {
        vehicle_state_pub.publish(vehicle_ready);
    }
}

void master_state_cb(std::shared_ptr<EmergencyServer> &emergency_server)
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
}

void system_state_cb(std::vector<std::shared_ptr<MonNodeMonitor>> &node_monitors,
                     ros::master::V_TopicInfo& master_topics,
                     std::vector<std::string>& mon_topics, 
                     ros::Publisher& abort_pub)
{
    ros::master::getTopics(master_topics);
    std::string mon_topic = "state";
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
    {
        const ros::master::TopicInfo &topic_i = *it;
        // Find rosmon topics
        if (topic_i.datatype == "rosmon_msgs/State")
        {
            // If the topic isn't already in the list, add it for monitoring
            if (std::find(mon_topics.begin(), mon_topics.end(), topic_i.name) == mon_topics.end())
            {
                // The freq of rosmon is 1hz, but multiply x 0.9 to account for slow transmission at times
                node_monitors.emplace_back(new MonNodeMonitor(topic_i.name, 1. * 0.9, abort_pub));
                mon_topics.push_back(topic_i.name);
            }
        }
    }
}

int main(int argn, char *args[])
{
    ros::init(argn, args, "emergency_manager");
    ros::NodeHandle nh("~");

    // Emergency surface action server
    std::shared_ptr<EmergencyServer> emergency_server(new EmergencyServer(nh));

    // Basic topics monitors: check that data is coming in on the defined topics. Signals an emergency when it stops
    std::string emerg_config, vehicle_ready_top, goto_wp_action, emergency_as;
    double vehicle_ready_period, master_check_period, nodes_check_period;

    nh.param<std::string>(("emergency_config_file"), emerg_config, "/home/torroba/catkin_workspaces/smarc_ws/src/smarc_missions/sam_action_servers/config/emergency_config.yaml");
    boost::filesystem::path emerg_config_path(emerg_config);
    if (!boost::filesystem::exists(emerg_config_path))
    {
        ROS_ERROR_STREAM("Emergency manager config file doesn't exist " << emerg_config_path.string());
        exit(0);
    }

    nh.param<std::string>(("goto_wp_action"), goto_wp_action, "goto_wp_action");
    actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> ac_g2wp(goto_wp_action, true);
    nh.param<std::string>(("emergency_as"), emergency_as, "emergency_as");
    actionlib::SimpleActionClient<smarc_bt::GotoWaypointAction> ac_emerg(emergency_as, true);
    while (!ac_g2wp.waitForServer() && !ac_emerg.waitForServer() && ros::ok())
    {
        ROS_WARN("Waiting for servers ");
        ros::Duration(1.).sleep();
    }
    ROS_INFO("Emergency node: action servers instantiated");

    std::string abort_top;
    nh.param<std::string>(("abort_top"), abort_top, "core/abort");
    ros::Publisher abort_pub = nh.advertise<std_msgs::Empty>(abort_top, 10);

    YAML::Node config = YAML::LoadFile(emerg_config_path.string());
    std::string robot_name = config["namespace"].as<std::string>();
    const YAML::Node &monitor = config["monitor"];
    std::vector<std::shared_ptr<GenericSensorMonitor>> sensor_monitors;
    for (YAML::const_iterator it = monitor.begin(); it != monitor.end(); ++it)
    {
        YAML::Node value = it->begin()->second;
        if (value.Type() == YAML::NodeType::Map)
        {
            std::string topic_name = value["name"].as<std::string>();
            double freq = value["freq"].as<double>();
            std::string topic = "/" + robot_name + "/" + topic_name;
            ros::Duration(0.01).sleep();
            sensor_monitors.emplace_back(new GenericSensorMonitor(topic, freq * 0.9, ac_g2wp, ac_emerg, abort_pub)); // Decrease slightly freq to account for slow transmission at times
        }
    }

    // Handle leak or abort messages, equally and only once
    bool emergency_trigger = false;
    boost::function<void(const std_msgs::Empty::ConstPtr &)> abort_cb = [&ac_g2wp, &ac_emerg, &emergency_trigger]
    (const std_msgs::Empty::ConstPtr &msg)
    {
        handle_abort(msg, ac_g2wp, ac_emerg, emergency_trigger);
    };

    boost::function<void(const smarc_msgs::Leak::ConstPtr &)> leak_cb = [&ac_g2wp, &ac_emerg, &emergency_trigger, &abort_pub]
    (const smarc_msgs::Leak::ConstPtr &msg)
    {
        handle_leak(msg, ac_g2wp, ac_emerg, abort_pub, emergency_trigger);
    };

    // Monitor state of the vehicle to let the BT know when it's ready for a mission
    nh.param<std::string>(("vehicle_ready_top"), vehicle_ready_top, "vehicle_ready");
    nh.param<double>(("vehicle_ready_period"), vehicle_ready_period, 1.);
    ros::Publisher vehicle_state_pub = nh.advertise<std_msgs::Bool>(vehicle_ready_top, 1);
    boost::function<void(const ros::TimerEvent &)> timer_cb = [&sensor_monitors, &vehicle_state_pub](const ros::TimerEvent &)
    {
        vehicle_state_cb(sensor_monitors, vehicle_state_pub);
    };
    ros::Timer vehicle_state_timer = nh.createTimer(ros::Duration(vehicle_ready_period), timer_cb);

    // Monitor state of ROS master. If down, trigger manual emergency behaviour
    boost::function<void(const ros::TimerEvent&)> master_cb = [&emergency_server](const ros::TimerEvent&)
    {
        master_state_cb(emergency_server);
    };
    nh.param<double>(("master_check_period"), master_check_period, 1.);
    ros::Timer master_state_timer = nh.createTimer(ros::Duration(master_check_period), master_cb);

    std::vector<std::shared_ptr<MonNodeMonitor>> node_monitors;
    ros::master::V_TopicInfo master_topics;
    std::vector<std::string> mon_topics;
    boost::function<void(const ros::TimerEvent &)> mon_timer_cb = [&node_monitors, &master_topics, &mon_topics, &abort_pub]
    (const ros::TimerEvent &)
    {
        system_state_cb(node_monitors, master_topics, mon_topics, abort_pub);
    };
    nh.param<double>(("nodes_check_period"), nodes_check_period, 2.);
    ros::Timer nodes_monitor_timer = nh.createTimer(ros::Duration(nodes_check_period), mon_timer_cb);

    std::string leak_top;
    nh.param<std::string>(("leak_top"), leak_top, "leak");
    ros::Subscriber leak_subs = nh.subscribe(leak_top, 1, leak_cb);
    ros::Subscriber abort_subs = nh.subscribe(abort_top, 1, abort_cb);

    ros::spin();

    return 0;
}
