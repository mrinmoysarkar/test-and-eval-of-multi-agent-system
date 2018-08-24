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
#include <cstdio>

using namespace std;

//Global Variable
double xPose;
double yPose;
double zPose;
double roll_angle;
double pitch_angle;
double yaw_angle;

double del = 0.1;
double del_time = 15.0;


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
    //cout << quatx << " " << quaty << " " << " " << quatz << " " << quatw << endl;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "executeScenario_node");
    ros::NodeHandle nh;

    //Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::Subscriber position_sub = nh.subscribe("mavros/local_position/pose", 100, pose_cb);

    //Publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 100);

    //Service Clients
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
      ("mavros/cmd/land");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");



    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ros::Time last_request = ros::Time::now();

    // change to offboard mode and arm
    while(ros::ok() && !current_state.armed){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
          ROS_INFO(current_state.mode.c_str());
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {

            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    
	ifstream inf("/home/mrinmoy/ros-intel-uav-rpeo/ros_ws/src/offbordctrl/src/scenario1.txt");
	if (!inf)
    {
        // Print an error and exit
        cerr << "Uh oh, scenario1.txt could not be opened for reading!" << endl;
        exit(1);
    }
    int totalPoint = 100;
    double x[totalPoint], y[totalPoint], z[totalPoint];
    double roll[totalPoint], pitch[totalPoint], yaw[totalPoint];
    int i = 0;
    while(true)
    {
	    inf >> x[i] >> y[i] >> z[i] >> roll[i] >> pitch[i] >> yaw[i];
        if(x[i] == -1)
        {
            break;
        }
        i++;
    }
    tf::Quaternion q_orig, q_rot, q_new;
    q_rot = tf::createQuaternionFromRPY(0, 0, 0);
    quaternionMsgToTF(pose.pose.orientation , q_orig);  // Get the original orientation of 'commanded_pose'
    q_new = q_rot*q_orig;  // Calculate the new orientation
    q_new.normalize();
    quaternionTFToMsg(q_new, pose.pose.orientation);  // Stuff the new rotation back into the pose. This requires conversion into a msg type
    int j = 1;
    double previous_time = ros::Time::now().toSec();
    while(true)
    {
        /*
        char cur = getchar();
        if(cur == 'q')
        {
            break;
        }
        */

        double current_time = ros::Time::now().toSec();
        
        if((/*(abs(x[j]-xPose) <= del && abs(y[j]-yPose) <= del && abs(z[j]-zPose) <= del)||*/(current_time-previous_time) > del_time))
        {
            pose.pose.position.x = x[j];
            pose.pose.position.y = y[j];
            pose.pose.position.z = z[j];
            q_rot = tf::createQuaternionFromRPY(roll[j], pitch[j], yaw[j]);
            quaternionMsgToTF(pose.pose.orientation , q_orig);
            q_new = q_rot*q_orig;  
            q_new.normalize();
            quaternionTFToMsg(q_new, pose.pose.orientation);

            j++;
           //cout << x[j] << "  " << y[j] << "  " << z[j] << endl;
            previous_time = current_time;
        }
        if(j == i)      
        {
            break;
        }
        //Publishing position
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    } // end while loop

    //Start landing command
    ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      ROS_INFO("tring to land");
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
