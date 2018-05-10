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




#define FLIGHT_ALTITUDE 1.5f


class point
{
    public:
        double x;
        double y;
};

point trajectory[5000];

double h=1.5;
double dx=0.25;
double dy=0.5;
double length=20;
double width=10;

double curr_x=0;
double curr_y=0;

double del_x = 0.1;
double del_y = 0.1;

int indx_trajectory=0;

int indx_max;

void init_trajectory()
{
    int m=int(length/dx);
    int n=int(width/dy);
    int indx=0;
    for(int i=0;i<m;i++)
    {
        if(i%2==0)
        {
            for(int j=0;j<n;j++)
            {
                trajectory[indx].x=j*dx;
                trajectory[indx].y=i*dy;
                indx++;
            }
        }
        else
        {
            for(int j=n-1;j>=0;j--)
            {
                trajectory[indx].x=j*dx;
                trajectory[indx].y=i*dy;
                indx++;
            }
        }
    }
    indx_max = indx;
}



void localPoscallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double cx = (msg->pose).position.x;    
    double cy = (msg->pose).position.y;    
    double cz = (msg->pose).position.z;
    

    double quatx= msg->pose.orientation.x;
    double quaty= msg->pose.orientation.y;
    double quatz= msg->pose.orientation.z;
    double quatw= msg->pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    //m.getRPY(roll_angle, pitch_angle, yaw_angle);
    
    if(h!=0 && abs(trajectory[indx_trajectory].x - cx)<del_x && abs(trajectory[indx_trajectory].y - cy)<del_y)
    {
        indx_trajectory++;
    }
    if(indx_trajectory == indx_max)
    {
        h=0;
    }
    else
    {
        curr_x = trajectory[indx_trajectory].x;
        curr_y = trajectory[indx_trajectory].y;
    }
  
    
    //cout << "Z:" << z << " roll:" << roll_angle << " pitch:" << pitch_angle << " yaw:" << yaw_angle << endl;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    init_trajectory();
    ros::init(argc, argv, "pub_trajectory_node");
    ros::NodeHandle nh;

    ros::Subscriber local_pos_sub = nh.subscribe("/mavros/local_position/pose",1000,localPoscallback);


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
    ros::Rate rate(250.0);

    // wait for FCU connection
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

    ROS_INFO("going to the first way point");
    while(true){
      pose.pose.position.x = curr_x;
      pose.pose.position.y = curr_y;
      pose.pose.position.z = h;

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
      if(h==0)
      {
         break;
      }
    }

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
