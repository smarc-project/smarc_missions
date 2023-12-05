#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <std_msgs/Bool.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TwistStamped.h"

#include "ros_type_introspection/ros_introspection.hpp"
#include <topic_tools/shape_shifter.h>
#include <thread>

using namespace RosIntrospection;
using topic_tools::ShapeShifter;

class GenericSensorMonitor
{

public:

    std::string topic_name_;
    double rate_;
    ros::CallbackQueue queue;
    ros::Subscriber subscriber_;

    GenericSensorMonitor(std::string &topic, double rate): topic_name_(topic), rate_(rate)
    {
        ros::NodeHandle nh;
        nh.setCallbackQueue(&queue);
        subscriber_ = nh.subscribe(topic_name_, 10, &GenericSensorMonitor::topicCB, this);
        std::thread(&GenericSensorMonitor::CheckQueue, this).detach();
    }

    void CheckQueue()
    {
        ros::Rate r(rate_);
        bool queue_init = false;
        while(ros::ok())
        {
            if(!queue.isEmpty())
            {
                queue.callOne(ros::WallDuration(0.01));
                ROS_INFO_STREAM("Callback " << topic_name_);
                queue_init = true;
            }
            else
            {
                if(queue_init)
                {
                    ROS_ERROR_STREAM("Data not coming in " << topic_name_);
                }
                // break;
            }
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
    ros::init(argn, args, "arm_to_fpga");
    std::string topic = "/bool/test";
    std::shared_ptr<GenericSensorMonitor> monitor(new GenericSensorMonitor(topic, 10.));
    std::string topic2 = "/bool/test2";
    std::shared_ptr<GenericSensorMonitor> monitor2(new GenericSensorMonitor(topic2, 100.));

    ros::spin();

    return 0;
}