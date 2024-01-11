#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include <thread>
#include <std_msgs/Empty.h>

using namespace tf;
using namespace ros;
using namespace std;

namespace smarc
{

class TFMonitor
{
public:
    std::string framea_, frameb_;
    bool using_specific_chain_;

    ros::NodeHandle *node_;
    ros::Subscriber subscriber_tf_, subscriber_tf_static_;
    ros::Publisher abort_pub_;
    std::vector<std::string> chain_;
    std::vector<std::string> chain_top_, chain_bottom_;
    std::vector<std::string> current_chain_;
    std::map<std::string, std::string> frame_authority_map;
    std::map<std::string, std::vector<double>> delay_map;
    std::map<std::string, std::vector<double>> authority_map;
    std::map<std::string, std::vector<double>> authority_frequency_map;
    ros::CallbackQueue queue_;

    TransformListener tf_;

    tf::tfMessage message_;

    std::mutex map_lock_;

    TFMonitor(ros::Publisher &abort_pub, std::string framea = "", std::string frameb = "");

    void callback(const ros::MessageEvent<tf::tfMessage const> &msg_evt);

    void static_callback(const ros::MessageEvent<tf::tfMessage const> &msg_evt);

    void process_callback(const tf::tfMessage &message, const std::string &authority, bool is_static);
    
    std::string outputFrameInfo(const std::map<std::string, std::vector<double>>::iterator &it, const std::string &frame_authority);

    void spin();
};
}