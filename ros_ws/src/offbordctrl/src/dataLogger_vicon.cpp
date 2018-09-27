// author: Mrinmoy Sarkar
// email: mrinmoy.pol@gmail.com
// date: 9/27/2018

#include <cstdlib>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <fstream>


using namespace std;

bool stop = false;
//Global Variable
double cur_x, cur_y, cur_z;
double prev_x, prev_y,prev_z;
double vx, vy, vz;
double cur_roll, cur_pitch, cur_yaw;
double prev_roll, prev_pitch, prev_yaw;
double vroll, vpitch, vyaw;

ofstream file;

double previous_time = 0.0;

void vicon_cb(const geometry_msgs::TransformStamped::ConstPtr& msg){
    double current_time = msg->header.stamp.sec + (msg->header.stamp.nsec)*(10e-9);
    double dt = current_time - previous_time;
    cur_x = msg->transform.translation.x;
    cur_y = msg->transform.translation.y;
    cur_z = msg->transform.translation.z;
    double quatx= msg->transform.rotation.x;
    double quaty= msg->transform.rotation.y;
    double quatz= msg->transform.rotation.z;
    double quatw= msg->transform.rotation.w;
    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(cur_roll, cur_pitch, cur_yaw);
    if(previous_time != 0.0)
    {
        vx = (cur_x-prev_x)/dt;
        vy = (cur_y-prev_y)/dt;
        vz = (cur_z-prev_z)/dt;
        vroll = (cur_roll-prev_roll)/dt;
        vpitch = (cur_pitch-prev_pitch)/dt;
        vyaw = (cur_yaw-prev_yaw)/dt;
        
        file << current_time << "," 
             << cur_x << ", " << cur_y << ", " << cur_z << ", "
             << cur_roll << ", " << cur_pitch << ", " << cur_yaw << ", "
             << vx << ", " << vy << ", " << vz << ", " 
             << vroll << ", " << vpitch << ", "<< vyaw << endl;   
    }

    previous_time = current_time;
    prev_x = cur_x;
    prev_y = cur_y;
    prev_z = cur_z;
    prev_roll = cur_roll; 
    prev_pitch = cur_pitch;
    prev_yaw = cur_yaw;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_dataLogger_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/Drone1/Drone1", 100, vicon_cb);

    double secs =ros::Time::now().toSec();
    long sec = long(secs);

    std::string time = std::to_string(sec);
    std::string fileName = "./src/offbordctrl/flightData/flight_path_data" + time + ".csv";

    cout<<fileName<<endl;
    
    file.open(fileName);
    ros::spin();
    file.close();
    cout << "\n stopping \n";
return 0;
}

