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
#include <stdio.h>

using namespace std;

// Global Variables
double x;
double y;
double z;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/*
void orient_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){
    double x = msg -> pose.orientation.x;
    double y = msg -> pose.orientation.y;
    double z = msg -> pose.orientation.z;
    double w = msg -> pose.orientation.w;
    //ROS_INFO("x: %f, y: %f, z: %f, w: %f", x, y, z, w);
}
*/

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    z = msg->pose.position.z;
    //ROS_INFO("x: %f, y: %f, z: %f", x, y, z);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Test_node");
    ros::NodeHandle nh;

    //Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::Subscriber position_sub = nh.subscribe("mavros/local_position/pose", 100, pose_cb);

    //ros::Subscriber orientation_sub = nh.subscribe("mavros/local_position/pose", 100, orient_cb);

    //Publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    //ros::Publisher local_orient_pub = nh.advertise<geometry_msgs::PoseStamped>
    //        ("mavros/local_position/pose", 10);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity",10);

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

    //geometry_msgs::TwistStamped velocity;
    /*
    velocity.twist.linear.x = 0;
    velocity.twist.linear.y = 0;
    velocity.twist.linear.z = 1;
    velocity.twist.angular.x = 0;
    velocity.twist.angular.y = 0;
    velocity.twist.angular.z = 1;
    */
    //geometry_msgs::PoseStamped orient;


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
/*
    void (*ptr)()=&pose_cb;

    B(ptr);
*/
    int i = 0;

    while(true){

     //Hovering State for 5 seconds

        //Hovering State for 5 seconds
        if (i<=10*20){
           //cout<< " Hovering "<< endl;
           pose.pose.position.x = 0;
           pose.pose.position.y = 0;
           pose.pose.position.z = 1;
           i++;
    }
     /*
    // Go to State 1 within 5 seconds
     else if(i >= 5*20 && i <= 10*20){
        //cout<< "Going to state 1 "<< endl;
//         stateObject.state1();
        pose.pose.position.x = 2;
        pose.pose.position.y = 0;
        pose.pose.position.z = 1;
        i++;
    }
      //Go back home within 5 seconds
        else if(i >= 10*20 && i <= 15*20){
         //cout<< "Going to home " << endl;
         pose.pose.position.x = 0;
         pose.pose.position.y = 0;
         pose.pose.position.z = 1;
         i++;
     }
     */
     // break while loop
     else if(i >= 10*20){
      break;
     }

      //cout << "x: " << x << " y: " << y << " z: " << z << " w: " << w << endl;
      //Publishing position
      //cout << "i: " << i << endl;
      //i++;
        cout<<"x: "<<x<<" y: "<<y<<" z: "<<z<<endl;
        local_pos_pub.publish(pose);
        //local_vel_pub.publish(velocity);
        //local_orient_pub.publish(orient);
        ros::spinOnce();
        rate.sleep();

    } // end while loop

    //Start landing command
    ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      //local_pos_pub.publish(pose);
      ROS_INFO("tring to land");
      ros::spinOnce();
      rate.sleep();
    }
    //return 0;
}
