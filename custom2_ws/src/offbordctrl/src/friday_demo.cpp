#include <cstdlib>
#include <ros/ros.h>
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




double h=1.5;
double x[]={0, 1.2, 1.2, 0,   0,   1.2, 1.2, 0,   0,   0,   0,  0,0};
double y[]={0, 0,   1.2, 1.2, 2.4, 2.4, 3.6, 3.6, 4.8, 3.6, 1.8,0,0};
double hold_time = 7.0;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1000);
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
    pose.pose.position.z = h;

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
    double	previous_time = ros::Time::now().toSec();
    int indx = 0;
    while(ros::ok()){
      pose.pose.position.x = x[indx];
      pose.pose.position.y = y[indx];
      pose.pose.position.z = h;

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
	    double current_time = ros::Time::now().toSec();
      if(indx < 12 && (current_time-previous_time) > hold_time)
      {
         indx++;
         previous_time  = current_time;
         if(indx == 12)
         {
            h = 0.2;
         }
      }
      if(indx == 12 && (current_time-previous_time) > hold_time)
      {
      	break;
      }
    }

    ROS_INFO("tring to land");
    int count = 0;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    while (!(land_client.call(land_cmd) && land_cmd.response.success) && count <= 15)
    {
      //local_pos_pub.publish(pose);
      ROS_INFO("tring to land");
      count++;
      ros::spinOnce();
      rate.sleep();
    }


/*
    last_request = ros::Time::now();
    arm_cmd.request.value = false;
    while(ros::ok() && current_state.armed)
    {
      if( current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle dis armed");
          break;
        }
        last_request = ros::Time::now();
      }
      ros::spinOnce();
      rate.sleep();
    }
*/
    for(int i=0;i<100;i++)
    {
      if(ros::ok())
      {
        ros::spinOnce();
        rate.sleep();
      }
    }
    arm_cmd.request.value = false;
    last_request = ros::Time::now();
    while(ros::ok() && current_state.armed)
    {
        if( current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle disarmed");
		            break;
            }
            last_request = ros::Time::now();
        }
    }
    return 0;
}
