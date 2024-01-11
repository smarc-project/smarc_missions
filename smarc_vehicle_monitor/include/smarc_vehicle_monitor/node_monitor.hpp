#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <rosmon_msgs/State.h>
#include <rosmon_msgs/NodeState.h>
#include <std_msgs/Empty.h>

#include <thread>

class MonNodeMonitor
{
public:
    std::string topic_name_, topic_end_;
    double rate_;
    ros::CallbackQueue queue_;
    ros::Subscriber subscriber_;
    ros::Publisher abort_pub_;
    ros::NodeHandle *nh_;
    bool node_up_, emergency_;
    std::map<std::string, int> nodes_;
    
    MonNodeMonitor(std::string topic_name, double rate, ros::Publisher& abort_pub);
    void emergency_detected(std::string name);
    void MonitorNodes();
    void MonCB(const rosmon_msgs::State::ConstPtr &state_msg);
};