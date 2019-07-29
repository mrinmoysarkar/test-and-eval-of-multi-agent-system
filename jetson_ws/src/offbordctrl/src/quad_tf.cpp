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

double roll_angle=0;
double pitch_angle=0;
double yaw_angle=0;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){
    double quatx= msg->pose.orientation.x;
    double quaty= msg->pose.orientation.y;
    double quatz= msg->pose.orientation.z;
    double quatw= msg->pose.orientation.w;


    //transform quaternion
    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll_angle, pitch_angle, yaw_angle);
    roll_angle*=180/3.14;
    pitch_angle*=180/3.14;
    yaw_angle*=180/3.14;
    cout << "Roll: " << roll_angle << ", Pitch: " << pitch_angle << ", Yaw: " << yaw_angle << endl;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "quad_tf_node");
    ros::NodeHandle nh;

    // Subcriber
    ros::Subscriber position_sub = nh.subscribe("mavros/local_position/pose", 100, pose_cb);

    ros::spin();

    return 0;

} // end main
