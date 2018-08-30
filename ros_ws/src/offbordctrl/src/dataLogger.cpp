// author: Mrinmoy Sarkar
// email: mrinmoy.pol@gmail.com
// date: 8/24/2018

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
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <fstream>

using namespace std;

//Global Variable
double xPose;
double yPose;
double zPose;
double xOrient;
double yOrient;
double zOrient;
double wOrient;
double vel_xLineTwist;
double vel_yLineTwist;
double vel_zLineTwist;
double vel_xAngTwist;
double vel_yAngTwist;
double vel_zAngTwist;
double imu_xOrient;
double imu_yOrient;
double imu_zOrient;
double imu_wOrient;
double imu_xAngVel;
double imu_yAngVel;
double imu_zAngVel;
double imu_xLinAcc;
double imu_yLinAcc;
double imu_zLinAcc;
double roll_angle;
double pitch_angle;
double yaw_angle;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


void pose_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){
    xPose = msg->pose.position.x;
    yPose = msg->pose.position.y;
    zPose = msg->pose.position.z;
    double quatx= msg->pose.orientation.x;
    double quaty= msg->pose.orientation.y;
    double quatz= msg->pose.orientation.z;
    double quatw= msg->pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll_angle, pitch_angle, yaw_angle);
}

void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr & msg){
    vel_xLineTwist = msg -> twist.linear.x;
    vel_yLineTwist = msg -> twist.linear.y;
    vel_zLineTwist = msg -> twist.linear.z;
    vel_xAngTwist = msg -> twist.angular.x;
    vel_yAngTwist = msg -> twist.angular.y;
    vel_zAngTwist = msg -> twist.angular.z;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr & msg){
    imu_xOrient = msg -> orientation.x;
    imu_yOrient = msg -> orientation.y;
    imu_zOrient = msg -> orientation.z;
    imu_wOrient = msg -> orientation.w;
    imu_xAngVel = msg -> angular_velocity.x;
    imu_yAngVel = msg -> angular_velocity.y;
    imu_zAngVel = msg -> angular_velocity.z;
    imu_xLinAcc = msg -> linear_acceleration.x;
    imu_yLinAcc = msg -> linear_acceleration.y;
    imu_zLinAcc = msg -> linear_acceleration.z;
}


/*Gather data from:
 *  /mavros/altitude
    /mavros/local_position/odom
    /mavros/local_position/pose
    /mavros/local_position/velocity
    /mavros/imu/data
*/

int main(int argc, char **argv)
{

    ros::init(argc, argv, "dataLogger_node");
    ros::NodeHandle nh;

    //Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);

    ros::Subscriber position_sub = nh.subscribe("mavros/local_position/pose", 100, pose_cb);

   // ros::Subscriber orientation_sub = nh.subscribe("mavros/local_position/odom", 100, odom_cb);

    ros::Subscriber velocity_sub = nh.subscribe("/mavros/local_position/velocity",100, velocity_cb);

    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data",100,imu_cb);

    double secs =ros::Time::now().toSec();
    long sec = long(secs);

    std::string time = std::to_string(sec);
    std::string fileName = "/home/intel1/ros_repo/ros_ws/src/offbordctrl/flightData/flight_path_data" + time + ".csv";

    cout<<fileName<<endl;
    ros::Rate rate(120.0);


    //Creating text file onto the Home Page
    ofstream file;
    file.open(fileName);
    while(ros::ok())
    {

        uint64_t seconds =ros::Time::now().toNSec();

        //Writing data onto text file
        file << seconds << "," << xPose << ", " << yPose << ", " << zPose << ", "
           << roll_angle << ", " << pitch_angle << ", " << yaw_angle << ", "
           << vel_xLineTwist << ", " << vel_yLineTwist << ", " << vel_zLineTwist << ", " << vel_xAngTwist << ", " << vel_yAngTwist << ", "<< vel_zAngTwist << ", "
           << imu_xAngVel << ", " << imu_yAngVel << ", " << imu_zAngVel << ", "
           << imu_xLinAcc << ", " << imu_yLinAcc << ", " << imu_zLinAcc << endl;

        //Publishing position
        ros::spinOnce();
        rate.sleep();
    } // end while loop

    file.close();
return 0;
}

