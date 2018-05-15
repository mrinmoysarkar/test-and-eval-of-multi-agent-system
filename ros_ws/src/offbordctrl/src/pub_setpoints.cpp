/*
#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
<<<<<<< HEAD
#include <iostream>

using namespace std;


float altitude = 0.5;

void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  altitude = msg->data;
}
double x,y,z;
double ox,oy,oz,ow;
long counter = 0;
void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	x = 0;//msg->pose.position.x;
	y = 0;////msg->pose.position.y;
	double currentz = msg->pose.orientation.z;
    if(counter > 1000 && abs(currentz-1.3) < 0.1)
	{
		z = 0.3;
	}
	else
	{
		z = 1.3;//msg->pose.position.z;
	}
	ox = 0;//msg->pose.orientation.x;
	oy = 0;//msg->pose.orientation.y;
	oz = 0;//msg->pose.orientation.z;
	ow = 1;//msg->pose.orientation.w;
	//cout << x << endl;
	counter++;
}

=======
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>



using namespace std;

float alti = 0.0;

bool flag = false;
void callback(const std_msgs::Float64::ConstPtr& msg)
{
    cout << msg->data << endl;
    alti = msg->data;
    flag = true;
}


double z = 0;
double roll_angle = 0;
double pitch_angle = 0;
double yaw_angle = 0;

double zdata[]={2,4,6};
int indx=0;
void localPoscallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    z = (msg->pose).pose.position.z;
    double quatx= msg->pose.pose.orientation.x;
    double quaty= msg->pose.pose.orientation.y;
    double quatz= msg->pose.pose.orientation.z;
    double quatw= msg->pose.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll_angle, pitch_angle, yaw_angle);
    alti = zdata[indx];
    if(abs(alti-z) < 0.1)
    {
        indx++;
        indx%=3;
    }
    //cout << "Z:" << z << " roll:" << roll_angle << " pitch:" << pitch_angle << " yaw:" << yaw_angle << endl;
}


>>>>>>> 579c6cc6084ea5fe6de8ef3f89a84113ec58fe1f
int main(int argc, char **argv)
{
   ros::init(argc, argv, "pub_setpoints");
   ros::NodeHandle n;
 

<<<<<<< HEAD
	ros::Subscriber sub = n.subscribe("chattersarkar", 1000, chatterCallback);
	ros::Subscriber sub_pos = n.subscribe("mavros/local_position/pose", 1000, pos_callback);
=======
   ros::Subscriber sub = n.subscribe("chattersarkar",1000,callback);
   ros::Subscriber local_pos_sub = n.subscribe("/mavros/global_position/local",1000,localPoscallback);

>>>>>>> 579c6cc6084ea5fe6de8ef3f89a84113ec58fe1f

   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
   ros::Rate loop_rate(100);
   ros::spinOnce();
 
   geometry_msgs::PoseStamped msg;
   int count = 1;
     
        //PositionReciever qp;:
        //Body some_object;
        //qp.connect_to_server();
 
     x=y=z=ox=oy=oz=0;
		ow=1;
   while(ros::ok()){
       //some_object = qp.getStatus();
        // some_object.print();
        //printf("%f\n",some_object.position_x);
       if(flag)
       {
           //break;
       }
       msg.header.stamp = ros::Time::now();
       msg.header.seq=count;
       msg.header.frame_id = 1;
<<<<<<< HEAD
       msg.pose.position.x = x;//0.001*some_object.position_x;
       msg.pose.position.y = y;//0.001*some_object.position_y;
       msg.pose.position.z = z;//0.001*some_object.position_z;
       msg.pose.orientation.x = ox;
       msg.pose.orientation.y = oy;
       msg.pose.orientation.z = oz;
       msg.pose.orientation.w = ow;
=======
       msg.pose.position.x = 0.0;//0.001*some_object.position_x;
       msg.pose.position.y = 0.0;//0.001*some_object.position_y;
       msg.pose.position.z = alti;//0.001*some_object.position_z;
       msg.pose.orientation.x = 0;
       msg.pose.orientation.y = 0;
       msg.pose.orientation.z = 0;
       msg.pose.orientation.w = 1;
>>>>>>> 579c6cc6084ea5fe6de8ef3f89a84113ec58fe1f
 
       chatter_pub.publish(msg);
       ros::spinOnce();
       count++;
       loop_rate.sleep();
   }
   cout << "returning" << endl;
   return 0;
}
*/




#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#define FLIGHT_ALTITUDE 1.0f


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
      ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(200.0);


    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

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

    // go to the first waypoint
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("going to the first way point");
    for(int i = 0; ros::ok() && i < 10*500; ++i){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("first way point finished!");


/*
    // go to the second waypoint
    pose.pose.position.x = 0;
    pose.pose.position.y = 1;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    //send setpoints for 10 seconds
    ROS_INFO("going to second way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("second way point finished!");

    // go to the third waypoint

    pose.pose.position.x = 1;
    pose.pose.position.y = 1;

    pose.pose.position.z = FLIGHT_ALTITUDE;
    //send setpoints for 10 seconds
    ROS_INFO("going to third way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("third way point finished!");

    // go to the forth waypoint
    pose.pose.position.x = 1;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;
    //send setpoints for 10 seconds
    ROS_INFO("going to forth way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("forth way point finished!");

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;
    ROS_INFO("going back to the first point!");
    //send setpoints for 10 seconds
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }

*/

    ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      //local_pos_pub.publish(pose);
      ROS_INFO("tring to land");
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}

