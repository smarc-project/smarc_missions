#include <ros/ros.h>

#include <smarc_msgs/DVL.h>
#include <smarc_msgs/Leak.h>
#include <smarc_msgs/ThrusterRPM.h>

#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Dense>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

class MissionSimulator {
private:

    //tf2_ros::Buffer tfBuffer;
    tf2_ros::StaticTransformBroadcaster staticBroadcaster;

    ros::NodeHandle nh_;

    smarc_msgs::DVL dvl_msg;
    sensor_msgs::FluidPressure pressure_msg;
    sensor_msgs::BatteryState battery_msg;

    ros::Publisher odom_pub;
    ros::Publisher dvl_pub;
    ros::Publisher pressure_pub;
    ros::Publisher battery_pub;

    ros::Subscriber depth_sub;
    ros::Subscriber yaw_sub;
    ros::Subscriber rpm1_sub;
    ros::Subscriber rpm2_sub;

    ros::Timer msg_timer;
    ros::Timer pos_timer;

    double time_delta;
    Eigen::Vector3d origin;
    Eigen::Vector3d pos;
    //Eigen::Quaterniond orient;
    Eigen::Matrix3d rot;
    double rpm1;
    double rpm2;
    double speed;
    double yaw_speed;
    double pitch_speed;

public:

    MissionSimulator()
    {
        odom_pub = nh_.advertise<nav_msgs::Odometry>("dr/odom", 1000);
        dvl_pub = nh_.advertise<smarc_msgs::DVL>("core/dvl", 1000);
        pressure_pub = nh_.advertise<sensor_msgs::FluidPressure>("core/pressure", 1000);
        battery_pub = nh_.advertise<sensor_msgs::BatteryState>("core/battery", 1000);

        time_delta = 0.01;

        msg_timer = nh_.createTimer(ros::Duration(0.1), &MissionSimulator::messagesCB, this);
        //pitch_timer = nh_.createTimer(ros::Duration(0.1), &MissionSimulator::pitch_controllerCB, this);
        //yaw_timer = nh_.createTimer(ros::Duration(0.1), &MissionSimulator::yaw_controllerCB, this);
        pos_timer = nh_.createTimer(ros::Duration(time_delta), &MissionSimulator::posCB, this);

        yaw_speed = 0.;
        pitch_speed = 0.;
        origin = Eigen::Vector3d(347158., 6573376., 0.);
        pos = origin; //.setZero();
        rot.setIdentity();
        rpm1 = 0.;
        rpm2 = 0.;

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.transform.translation.x = origin[0];
        transformStamped.transform.translation.y = origin[1];
        transformStamped.transform.translation.z = origin[2];
        transformStamped.transform.rotation.w = 1.;
        transformStamped.header.frame_id = "utm";
        transformStamped.child_frame_id = "map";
        transformStamped.header.stamp = ros::Time::now();
        staticBroadcaster.sendTransform(transformStamped);

        depth_sub = nh_.subscribe("ctrl/depth_setpoint", 1000, &MissionSimulator::depth_callback, this);
        yaw_sub = nh_.subscribe("ctrl/yaw_setpoint", 1000, &MissionSimulator::yaw_callback, this);
        rpm1_sub = nh_.subscribe("core/thruster1_cmd", 1000, &MissionSimulator::rpm1_callback, this);
        rpm2_sub = nh_.subscribe("core/thruster2_cmd", 1000, &MissionSimulator::rpm2_callback, this);
    }

    void depth_callback(const std_msgs::Float64& depth)
    {
        double z_offset = -depth.data - (pos[2] + 10.*rot(2, 0)); // offset 10 m in front
        ROS_INFO("Z offset: %f, depth: %f", z_offset, depth.data);
        double max_pitch_speed = 0.1;
        double max_pitch = M_PI/4.;
        // 0.1 radians per 1m offset
        pitch_speed = std::max(-max_pitch_speed, std::min(max_pitch_speed, -.01*z_offset/1.));
        ROS_INFO("Pitch speed: %f", pitch_speed);

        //Eigen::Vector3d x = rot.col(0);
        //Eigen::Vector3d x_plane = x;
        //x_plane(2) = 0.;
        //x_plane.normalize();

        double pitch = acos(std::min(1., rot.col(0).head<2>().norm())); // should be sqrt(x**2 + y**2) ?
        ROS_INFO("Pitch: %f", pitch);
        if ((rpm1 == 0 && rpm2 == 0) || (pitch > max_pitch && pitch*pitch_speed > 0.)) {
            pitch_speed = 0.;
        }
    }

    void rpm1_callback(const smarc_msgs::ThrusterRPM& rpm)
    {
        rpm1 = rpm.rpm;
    }

    void rpm2_callback(const smarc_msgs::ThrusterRPM& rpm)
    {
        rpm2 = rpm.rpm;
    }

    void yaw_callback(const std_msgs::Float64& yaw)
    {
        if (rpm1 == 0 && rpm2 == 0) {
            yaw_speed = 0.;
            return;
        }

        double max_yaw_speed = 0.1;
        Eigen::Vector2d goal_dir(cos(yaw.data), sin(yaw.data));
        Eigen::Vector2d vehicle_frame = rot.topLeftCorner<2, 2>().transpose()*goal_dir;
        double yaw_offset = atan2(vehicle_frame[1], vehicle_frame[0]);
        // 0.1 radians per PI radian offset
        yaw_speed = std::max(-max_yaw_speed, std::min(max_yaw_speed, 2.*yaw_offset/M_PI));
    }

    void messagesCB(const ros::TimerEvent& e)
    {
        nav_msgs::Odometry odom;
        odom.pose.pose.position.x = pos[0]-origin[0];
        odom.pose.pose.position.y = pos[1]-origin[1];
        odom.pose.pose.position.z = pos[2]-origin[2];
        Eigen::Quaterniond quat(rot);
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        odom_pub.publish(odom);

        dvl_msg.altitude = pos[2] - -20.; // constant 20m depth
        dvl_pub.publish(dvl_msg);
        pressure_pub.publish(pressure_msg);
        battery_pub.publish(battery_msg);
    }

    void posCB(const ros::TimerEvent& e)
    {
        double speed = .5*(rpm1+rpm2)/500.;

        //ROS_INFO("Speed: %f, rpm, rpm: %f, %f", speed, rpm1, rpm2);
        pos.array() += speed*time_delta*rot.col(0).array();
        if (pos[2] > 0.) {
            pos[2] = 0.;
        }
        Eigen::Matrix3d speed_rot;
        speed_rot = Eigen::AngleAxisd(pitch_speed*time_delta, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(yaw_speed*time_delta, Eigen::Vector3d::UnitZ());
        rot *= speed_rot;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_sim_node");

    MissionSimulator sim;

    ros::spin();

    return 0;
}
