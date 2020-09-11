#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>
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


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


int main(int argc, char **argv)
{


    ros::init(argc, argv, "setHome_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::ServiceClient set_home_client = nh.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }
    ROS_INFO("connected to FCT...");
    mavros_msgs::CommandHome homeset_cmd;
    homeset_cmd.request.current_gps = true;
    homeset_cmd.request.latitude = 0;
    homeset_cmd.request.longitude = 0;
    homeset_cmd.request.altitude = 0;

    ros::Time last_request = ros::Time::now();
    while(ros::ok())
    {
        if((ros::Time::now() - last_request > ros::Duration(5.0)))
        {
             if( set_home_client.call(homeset_cmd) && homeset_cmd.response.success)
             {
                    ROS_INFO("home seted");
                    break;
             }
             last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
