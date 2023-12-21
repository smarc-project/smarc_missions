#include "smarc_vehicle_monitor/node_monitor.hpp"

MonNodeMonitor::MonNodeMonitor(std::string topic_name, double rate, ros::Publisher& abort_pub) : 
    topic_name_(topic_name), rate_(rate), abort_pub_(abort_pub)
{
    nh_ = new ros::NodeHandle("~");
    nh_->setCallbackQueue(&queue_);

    ROS_INFO_STREAM("Node monitor: listening to " << topic_name_);
    subscriber_ = nh_->subscribe(topic_name_, 1, &MonNodeMonitor::MonCB, this);
    node_up_ = false;

    std::thread(&MonNodeMonitor::MonitorNodes, this).detach();
}

void MonNodeMonitor::MonitorNodes()
{
    ros::Rate r(rate_);
    while (ros::ok())
    {
        // If msg in queue
        if (!queue_.isEmpty())
        {
            // rosmon publishes the state at 1hz
            queue_.callOne(ros::WallDuration(0.01));
            node_up_ = true;
        }
        // If the queue is empty:
        else
        {
            // If the queue is empty but it shouldn't be, emergency
            if (node_up_)
            {
                this->emergency_detected("mon launch session " + topic_name_);
            }
            // If the queue is empty because the data flow has not started yet, throw warning
            else
            {
                ROS_WARN_STREAM_THROTTLE(int(rate_), "Node monitor: data stream not initialized " << topic_name_);
            }
        }
        r.sleep();
    }
}

void MonNodeMonitor::MonCB(const rosmon_msgs::State::ConstPtr& state_msg)
{
    // If there's a new node in the mon session, add to list
    for(const rosmon_msgs::NodeState& node_i : state_msg->nodes)
    {
        // Emplace will return false if there already exists the node in the list
        auto result = nodes_.emplace(node_i.ns + "/" + node_i.name, nodes_.size());
        if (result.second)
        {
            std::cout << "Node monitor: monitoring " << node_i.ns + "/" + node_i.name << std::endl;
        }

        // std::cout << (result.second ? "Inserted: " : "ignored: ") << node_i.ns + "/" + node_i.name << std::endl;
    }

    // Check state of all nodes in the list
    for (const rosmon_msgs::NodeState &node : state_msg->nodes)
    {
        // TODO: add logic to try to relaunch nodes and count attempts before emergency
        // Send nodes statistics back to main node for resources monitoring
        switch (int(node.state))
        {
            case node.IDLE:
                ROS_DEBUG_STREAM("Node " << node.name << " idle");
                break;
            case node.RUNNING:
                ROS_DEBUG_STREAM("Node " << node.name << " running");
                break;
            case node.CRASHED:
                this->emergency_detected("node " + node.name);
                break;
            case node.WAITING:
                ROS_DEBUG_STREAM("Node " << node.name << " waiting");
                break;
        }
    }
}

void MonNodeMonitor::emergency_detected(std::string name)
{
    ROS_ERROR("-------------------------------------------------------------------");
    ROS_ERROR_STREAM("Rosmon monitor: " << name << " crahed, aborting mission");
    ROS_ERROR("-------------------------------------------------------------------");
    std_msgs::Empty abort;
    abort_pub_.publish(abort);
    node_up_ = false; // And reset monitor
}