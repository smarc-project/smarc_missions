#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Float64.h>
#include <smarc_msgs/ThrusterRPM.h>
#include <smarc_msgs/GotoWaypointAction.h>

#include <eigen3/Eigen/Dense>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

class WaypointMissionServer {
private:

    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    std::string robot_name;
    ros::Publisher rpm1_pub;
    ros::Publisher rpm2_pub;
    ros::Publisher depth_pub;
    ros::Publisher yaw_pub;
    actionlib::SimpleActionServer<smarc_msgs::GotoWaypointAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.

    std::string action_name_;
    // create messages that are used to published feedback/result
    smarc_msgs::GotoWaypointFeedback feedback_;
    smarc_msgs::GotoWaypointResult result_;

public:

    WaypointMissionServer(const std::string& name) :
        tfListener(tfBuffer), as_(nh_, name, boost::bind(&WaypointMissionServer::executeCB, this, _1), false),
        action_name_(name)
    {
        ros::NodeHandle pn("~");
        pn.param<std::string>("robot_name", robot_name, "lolo");
        depth_pub = nh_.advertise<std_msgs::Float64>("ctrl/depth_setpoint", 1000);
        yaw_pub = nh_.advertise<std_msgs::Float64>("ctrl/yaw_setpoint", 1000);
        rpm1_pub = nh_.advertise<smarc_msgs::ThrusterRPM>("core/thruster1_cmd", 1000);
        rpm2_pub = nh_.advertise<smarc_msgs::ThrusterRPM>("core/thruster2_cmd", 1000);
        as_.start();
    }

    void executeCB(const smarc_msgs::GotoWaypointGoalConstPtr& goal)
    {
        bool success = false;
        result_.reached_waypoint = false;

        Eigen::Vector3d goal_pos;
        goal_pos[0] = goal->waypoint_pose.pose.position.x;
        goal_pos[1] = goal->waypoint_pose.pose.position.y;
        goal_pos[2] = goal->waypoint_pose.pose.position.z;
        double speed = 1.;

        smarc_msgs::ThrusterRPM rpm;
        std_msgs::Float64 goal_yaw;
        std_msgs::Float64 travel_depth;
        travel_depth.data = goal->travel_depth;

        ros::Rate rate(.5); // 2hz
        while (ros::ok()) {

            geometry_msgs::TransformStamped transformStamped;
            try {
                transformStamped = tfBuffer.lookupTransform("utm", robot_name+"/base_link", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_INFO("Could not get base_link transform...");
                rate.sleep();
                continue;
            }
            Eigen::Vector3d pos;
            pos[0] = transformStamped.transform.translation.x;
            pos[1] = transformStamped.transform.translation.y;
            pos[2] = transformStamped.transform.translation.z;

            double dist = (goal_pos - pos).head<2>().norm();
            if (dist < goal->goal_tolerance) {
                success = true;
                break;
            }

            Eigen::Vector3d direction = goal_pos - pos;
            goal_yaw.data = atan2(direction[1], direction[0]);
            yaw_pub.publish(goal_yaw);
            depth_pub.publish(travel_depth);

            rpm.rpm = 1000;
            rpm1_pub.publish(rpm);
            rpm2_pub.publish(rpm);

            ROS_INFO("Distance: %fm", dist);

            double eta_s = dist / speed;
            feedback_.ETA = ros::Time::now() + ros::Duration(eta_s);
            as_.publishFeedback(feedback_);
            rate.sleep();
            //ros::spinOnce();
        }

        rpm.rpm = 0;
        rpm1_pub.publish(rpm);
        rpm2_pub.publish(rpm);

        if (success) {
            result_.reached_waypoint = true;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_sim_waypoint_nav");

    WaypointMissionServer server("ctrl/goto_waypoint");

    ros::spin();

    return 0;
}
