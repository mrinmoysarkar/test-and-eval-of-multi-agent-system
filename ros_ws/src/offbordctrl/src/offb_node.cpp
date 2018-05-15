
#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ActuatorControl.h>
#include <std_msgs/Float64.h>
<<<<<<< HEAD
=======
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

int no_of_point = 1000;
std::vector<double> zpoint(no_of_point);
std::vector<double> rollpoint(no_of_point);
std::vector<double> pitchpoint(no_of_point);
std::vector<double> yawpoint(no_of_point);
int indexpoint = 0;

void plot() {
    plt::figure();
    plt::plot(zpoint);
    plt::figure();
    plt::plot(rollpoint);
    plt::figure();
    plt::plot(pitchpoint);
    plt::figure();
    plt::plot(yawpoint);
    plt::show();
}



>>>>>>> 579c6cc6084ea5fe6de8ef3f89a84113ec58fe1f

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double z = 0;
double roll_angle = 0;
double pitch_angle = 0;
double yaw_angle = 0;
void localPoscallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    z = (msg->pose).position.z;
    zpoint[indexpoint]=z;

    double quatx= msg->pose.orientation.x;
    double quaty= msg->pose.orientation.y;
    double quatz= msg->pose.orientation.z;
    double quatw= msg->pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll_angle, pitch_angle, yaw_angle);
    rollpoint[indexpoint] = roll_angle;
    pitchpoint[indexpoint] = pitch_angle;
    yawpoint[indexpoint] = yaw_angle;
    indexpoint++;
    //cout << "Z:" << z << " roll:" << roll_angle << " pitch:" << pitch_angle << " yaw:" << yaw_angle << endl;
}



int main(int argc, char **argv)
{
        //plot();
        ros::init(argc, argv, "mavros_takeoff");
	ros::NodeHandle n;


        ros::Rate rate(20.0);


        ros::Subscriber local_pos_sub = n.subscribe("/mavros/local_position/pose",1000,localPoscallback);
        ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;

        //send a few setpoints before starting

        for(int i = 100; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }



    ////////////////////////////////////////////
    /////////////////OFFBOARD/////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "OFFBOARD";
    if(cl.call(srv_setMode)){
        ROS_INFO("setmode send ok");// %d value:", srv_setMode.response.success);
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok");// %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }

/*
    ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 2;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_INFO("srv_takeoff send ok");// %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }
*/

/*
    ////////////////////////////////////////////
    /////////////////DO STUFF///////////////////
    ////////////////////////////////////////////
<<<<<<< HEAD
   
	//sleep(5);
	//ros::Publisher chetter_pub = n.advertise<std_msgs::Float64>("chattersarkar", 1000);
	//std_msgs::Float64 alti;
	//alti.data = 0.1;
	ros::Rate loop_rate(20);
	//for(int i=0;i<10;i++)
	while(ros::ok())
	{
		//chetter_pub.publish(alti);
		ros::spinOnce();
		loop_rate.sleep();	
	}


=======
    //sleep(10);
    ros::Publisher chetter_pub0 = n.advertise<std_msgs::Float64>("chattersarkar",1000);

    std_msgs::Float64 alti;
    alti.data = 0.7;
    ros::Rate loop_ratepub(2);
    for(int i=0;i<2;i++)
    while(ros::ok())
    {
        //chetter_pub0.publish(alti);
        ros::spinOnce();
        loop_ratepub.sleep();
    }
*/
>>>>>>> 579c6cc6084ea5fe6de8ef3f89a84113ec58fe1f
/*
    ////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 2;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok");// %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }
*/
<<<<<<< HEAD
    //sleep(5);
=======
    /*
    sleep(10);
>>>>>>> 579c6cc6084ea5fe6de8ef3f89a84113ec58fe1f
    ////////////////////////////////////////////
    ///////////////////DISARM//////////////////////
    ////////////////////////////////////////////
    //ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    //mavros_msgs::CommandBool srv;
/*
    srv.request.value = false;
    if(arming_cl.call(srv)){
        ROS_INFO("DISARM send ok");// %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }
    sleep(10);
*/

<<<<<<< HEAD
*/
    /*
    ros::Publisher chatter_pub = n.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control",100);
=======

    ros::Publisher chatter_pub = n.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control",1000);
>>>>>>> 579c6cc6084ea5fe6de8ef3f89a84113ec58fe1f

    mavros_msgs::ActuatorControl msgActrl;


    int count = 1;
    ros::Rate loop_rate(200);
    double kp = 0.3,kpr = 0.2,kpp = 0.2,kpy = 0.1;
    double errorz,errorx,errory,errorroll,errorpitch,erroryaw;
    double throttle;
    while(ros::ok() && indexpoint<no_of_point){
        errorz = 2-z;
        throttle = errorz*kp;

        throttle = throttle < 0?0:throttle>.8?.8:throttle;

        errorroll = 0-pitch_angle;
        errorpitch = 0-roll_angle;
        erroryaw = 0-yaw_angle;

        double roll=kpr*errorroll;
        double pitch=kpp*errorpitch;
        double yaw=kpy*erroryaw;

        roll = roll < -0.6?-0.6:roll>0.6?0.6:roll;
        pitch = pitch < -0.6?-0.6:pitch>0.6?0.6:pitch;
        yaw = yaw < -0.6?-0.6:yaw>0.6?0.6:yaw;

        msgActrl.header.stamp = ros::Time::now();
        msgActrl.header.seq=count;
        msgActrl.header.frame_id = 1;
        msgActrl.group_mix = msgActrl.PX4_MIX_FLIGHT_CONTROL;
        msgActrl.controls[0] = roll;//roll_angle<-.5?.5:roll_angle>.5?-0.5:-roll_angle;//roll (-1..1)
        msgActrl.controls[1] = pitch;//pitch_angle<-.5?.5:pitch_angle>.5?-0.5:-pitch_angle;//pitch (-1..1)
        msgActrl.controls[2] = yaw;//yaw_angle<-.5?-.5:yaw_angle>.5?0.5:yaw_angle;//yaw (-1..1)
        msgActrl.controls[3] = throttle;//throttle (0..1 normal range, -1..1 for variable pitch / thrust reversers)
        msgActrl.controls[4] = 0;//flaps (-1..1)
        msgActrl.controls[5] = 0;//spoilers (-1..1)
        msgActrl.controls[6] = 0;//airbrakes (-1..1)
        msgActrl.controls[7] = 0;//landing gear (-1..1)
        chatter_pub.publish(msgActrl);
        ros::spinOnce();
        count++;
        loop_rate.sleep();
    }
    indexpoint=0;
    plot();





    /*
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

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

    ros::Time last_request = ros::Time::now();

    int lop = 0;
    while(ros::ok() && lop<1000){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
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
        lop++;
    }
    
    //arm_cmd.request.value = false;
    //if( arming_client.call(arm_cmd) &&
      //  arm_cmd.response.success){
        //ROS_INFO("Vehicle disarmed");
    //}
    return 0;*/
}






