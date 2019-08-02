
#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ActuatorControl.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <vector>


using namespace std;


int no_of_point = 1000;
std::vector<double> zpoint(no_of_point);
std::vector<double> rollpoint(no_of_point);
std::vector<double> pitchpoint(no_of_point);
std::vector<double> yawpoint(no_of_point);
int indexpoint = 0;





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
    ros::init(argc, argv, "position_controller");
	ros::NodeHandle n;
    ros::Rate rate(20);
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1000);
    ros::ServiceClient land_client = n.serviceClient<mavros_msgs::CommandTOL>
      ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok");// %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }
/*
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

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "MANUAL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ros::Time last_request = ros::Time::now();

    
    while(ros::ok() && !current_state.armed)
    {
        if( current_state.mode != "MANUAL" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            ROS_INFO(current_state.mode.c_str());
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("MANUAL enabled");
            }
            last_request = ros::Time::now();
        } 
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

   */

    ros::Publisher chatter_pub = n.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control",1000);

    mavros_msgs::ActuatorControl msgActrl;


    int count = 1;
    ros::Rate loop_rate(30);
    double kp = 0.3,kpr = 0.2,kpp = 0.2,kpy = 0.1;
    double errorz,errorx,errory,errorroll,errorpitch,erroryaw;
    double motor1;
    double motor2;
    double motor3;
    double motor4;
    while(ros::ok()){
        //errorz = kp*(1-z);
        
        motor1 = 0;//range 0..1
        motor2 = 0;
        motor3 = 0;
        motor4 = 0;

        msgActrl.header.stamp = ros::Time::now();
        msgActrl.header.seq=count;
        msgActrl.header.frame_id = "base";
        msgActrl.group_mix = msgActrl.PX4_MIX_FLIGHT_CONTROL;
        msgActrl.controls[0] = motor1;//roll_angle<-.5?.5:roll_angle>.5?-0.5:-roll_angle;//roll (-1..1)
        msgActrl.controls[1] = motor2;//pitch_angle<-.5?.5:pitch_angle>.5?-0.5:-pitch_angle;//pitch (-1..1)
        msgActrl.controls[2] = motor3;//yaw_angle<-.5?-.5:yaw_angle>.5?0.5:yaw_angle;//yaw (-1..1)
        msgActrl.controls[3] = motor4;//throttle (0..1 normal range, -1..1 for variable pitch / thrust reversers)
        msgActrl.controls[4] = 0;//flaps (-1..1)
        msgActrl.controls[5] = 0;//spoilers (-1..1)
        msgActrl.controls[6] = 0;//airbrakes (-1..1)
        msgActrl.controls[7] = 0;//landing gear (-1..1)
        chatter_pub.publish(msgActrl);
        //local_pos_pub.publish(pose);
        ros::spinOnce();
        count++;
        loop_rate.sleep();
    }

    return 0;
}







