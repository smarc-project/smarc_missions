#include "smarc_vehicle_monitor/tf_monitor.hpp"

using namespace tf;
using namespace ros;
using namespace std;
using namespace smarc;


void TFMonitor::callback(const ros::MessageEvent<tf::tfMessage const> &msg_evt)
{
    const tf::tfMessage &message = *(msg_evt.getConstMessage());
    std::string authority = msg_evt.getPublisherName(); // lookup the authority
    process_callback(message, authority, false);
}

void TFMonitor::static_callback(const ros::MessageEvent<tf::tfMessage const> &msg_evt)
{
    const tf::tfMessage &message = *(msg_evt.getConstMessage());
    std::string authority = msg_evt.getPublisherName() + std::string("(static)"); // lookup the authority
    process_callback(message, authority, true);
}

void TFMonitor::process_callback(const tf::tfMessage &message, const std::string &authority, bool is_static)
{
    double average_offset = 0;
    // map_lock_.lock();
    for (unsigned int i = 0; i < message.transforms.size(); i++)
    {
        frame_authority_map[message.transforms[i].child_frame_id] = authority;

        double offset;
        if (is_static)
        {
            offset = 0.0;
        }
        else
        {
            offset = (ros::Time::now() - message.transforms[i].header.stamp).toSec();
        }
        average_offset += offset;

        std::map<std::string, std::vector<double>>::iterator it = delay_map.find(message.transforms[i].child_frame_id);
        if (it == delay_map.end())
        {
            delay_map[message.transforms[i].child_frame_id] = std::vector<double>(1, offset);
        }
        else
        {
            it->second.push_back(offset);
            if (it->second.size() > 1000)
                it->second.erase(it->second.begin());
        }
    }

    average_offset /= max((size_t)1, message.transforms.size());

    // create the authority log
    std::map<std::string, std::vector<double>>::iterator it2 = authority_map.find(authority);
    if (it2 == authority_map.end())
    {
        authority_map[authority] = std::vector<double>(1, average_offset);
    }
    else
    {
        it2->second.push_back(average_offset);
        if (it2->second.size() > 1000)
            it2->second.erase(it2->second.begin());
    }

    // create the authority frequency log
    std::map<std::string, std::vector<double>>::iterator it3 = authority_frequency_map.find(authority);
    if (it3 == authority_frequency_map.end())
    {
        authority_frequency_map[authority] = std::vector<double>(1, ros::Time::now().toSec());
    }
    else
    {
        it3->second.push_back(ros::Time::now().toSec());
        if (it3->second.size() > 1000)
            it3->second.erase(it3->second.begin());
    }
    // map_lock_.unlock();
}

TFMonitor::TFMonitor(ros::Publisher &abort_pub, std::string framea, std::string frameb) : framea_(framea), frameb_(frameb),
                                                                                        abort_pub_(abort_pub)
{
    node_ = new ros::NodeHandle("~");
    node_->setCallbackQueue(&queue_);

    subscriber_tf_ = node_->subscribe("/tf", 100, &TFMonitor::callback, this);
    subscriber_tf_static_ = node_->subscribe("/tf_static", 100, &TFMonitor::static_callback, this);

    std::thread(&TFMonitor::spin, this).detach();
}

std::string TFMonitor::outputFrameInfo(const std::map<std::string, std::vector<double>>::iterator &it, const std::string &frame_authority)
{
    std::stringstream ss;
    double average_delay = 0;
    double max_delay = 0;
    for (unsigned int i = 0; i < it->second.size(); i++)
    {
        average_delay += it->second[i];
        max_delay = std::max(max_delay, it->second[i]);
    }
    average_delay /= it->second.size();
    ss << "Frame: " << it->first << " published by " << frame_authority << " Average Delay: " << average_delay << " Max Delay: " << max_delay << std::endl;
    return ss.str();
}

void TFMonitor::spin()
{

    // create tf listener
    double max_diff = 0.;
    double diff_thres = 5.; 
    double avg_diff = 0;
    double lowpass = 0.01;
    unsigned int counter = 0;
    bool print = true;

    while (node_->ok())
    {
        if (!queue_.isEmpty())
        {
            queue_.callAvailable(ros::WallDuration(0.01));
        }

        tf::StampedTransform tmp;
        tmp.stamp_ = Time::now();
        counter++;

        try
        {
            if(tf_.waitForTransform(framea_, frameb_, Time(), Duration(0.1)))
            {
                tf_.chainAsVector(frameb_, ros::Time(), framea_, ros::Time(), frameb_, chain_);
                tf_.lookupTransform(framea_, frameb_, Time(), tmp);

                // Keep track of net delays once the tree is up
                double diff = (Time::now() - tmp.stamp_).toSec();
                avg_diff = lowpass * diff + (1 - lowpass) * avg_diff;
                if (diff > max_diff)
                    max_diff = diff;

                if (max_diff > diff_thres)
                {
                    ROS_ERROR("-------------------------------------------------------------------");
                    ROS_ERROR_STREAM("TF monitor: "
                                     << " tf tree has not been updated in " << max_diff << " seconds, emergency triggered");
                    ROS_ERROR("-------------------------------------------------------------------");

                    std_msgs::Empty abort_msg;
                    abort_pub_.publish(abort_msg);
                    return;
                }
            }
            else
            {
                std::string parent;
                std::string frame_prev;
                frame_prev = framea_;
                while (tf_.getParent(frame_prev, ros::Time(), parent))
                {
                    frame_prev = parent;
                }
                tf_.chainAsVector(frame_prev, ros::Time(), framea_, ros::Time(), frame_prev, chain_);
                tf_.lookupTransform(framea_, frame_prev, Time(), tmp);
                // ROS_WARN_STREAM("Current chain " << framea_ << " to " << frame_prev);
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Transform Exception %s", ex.what());
            // return;
        }


        if (counter > 2 && print)
        {
            if (frameb_ != chain_.back())
            {
                ROS_WARN_STREAM("Full tf tree " << framea_ << " to " << frameb_ << " isn't there yet");
                counter = 0;
            }
            else
            {
                ROS_INFO_STREAM("Full tf tree " << framea_ << " to " << frameb_ << " constructed!");
                print = false;
            }

            cout << "Current tree is: ";
            for (unsigned int i = 0; i < chain_.size(); i++)
            {
                cout << chain_[i];
                if (i != chain_.size() - 1)
                    cout << " -> ";
            }
            cout << std::endl;
            cout << "Net delay "
                << "    avg = " << avg_diff << ": max = " << max_diff << endl;
        }

        Duration(1.).sleep();
    }
}
