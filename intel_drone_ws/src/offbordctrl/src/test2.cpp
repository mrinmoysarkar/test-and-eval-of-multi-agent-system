#include <cstdlib>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
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

//Global Variables
double x;
double y;
double z;
double xOrient;
double yOrient;
double zOrient;
double wOrient;



mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


/*
mavros_msgs::Altitude altitude;
void altitude_cb(const mavros_msgs::Altitude::ConstPtr & msg){
    double alt = msg -> relative;
    ROS_INFO("Altitude is: %f", alt);
}
*/

void nav_cb(const nav_msgs::Odometry::ConstPtr & msg){
    xOrient = msg -> pose.pose.orientation.x;
    yOrient = msg -> pose.pose.orientation.y;
    zOrient = msg -> pose.pose.orientation.z;
    wOrient = msg -> pose.pose.orientation.w;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    z = msg->pose.position.z;
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

    ros::init(argc, argv, "test2_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber navSub = nh.subscribe("mavros/local_position/odom",10,nav_cb);

    //ros::Subscriber altitude_sub = nh.subscribe("mavros/altitude",10,altitude_cb);

    ros::Subscriber position_sub = nh.subscribe("mavros/local_position/pose", 1000, pose_cb);


    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);


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

    //Create a text file on Home page
    ofstream file;
    file.open("example.txt");
    file << "[xPose]   [yPose]   [zPose]   [xOrient]   [yOrient]   [zOrient]   [wOrient] \n";

    int i = 0;
    while(true){
     // Hover
     if (i<=5*20){
        //cout <<"Taking Off"<<endl;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 1;
        i++;
    }

     // State 1
     else if(i >= 5*20 && i <= 10*20){
        //cout <<"Going to State1"<<endl;
        pose.pose.position.x = 1;
        pose.pose.position.y = 0;
        pose.pose.position.z = 1;
        i++;
     }
     // Home
     else if(i >= 10*20 && i <= 15*20){
         //cout <<"Going home"<<endl;
         pose.pose.position.x = 0;
         pose.pose.position.y = 0;
         pose.pose.position.z = 1;
         i++;
     }

     // Break while loop
     else {
        break;
     }

     //Writing data into the text file
     file << x << ", " << y << ", " << z << ", "
          << xOrient << ", " << yOrient << ", " << zOrient << ", " << wOrient << endl;

      //cout <<"x: "<< x << " y: " << y << " z: " << z << endl;
     //Publish position
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();

    } //while loop end

    //Landing command
    ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      //local_pos_pub.publish(pose);
      ROS_INFO("tring to land");
      ros::spinOnce();
      rate.sleep();
    }

    //Close text file
    file.close();


    //return 0;
}
