#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/ParamGet.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <vector>


using namespace std;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_takeoff");
	ros::NodeHandle n;


    ros::Rate rate(20.0);
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
    mavros_msgs::ParamGet srv_getParam;
    srv_getParam.request.param_id = "BAT_A_PER_V";
    if(cl.call(srv_getParam) && srv_getParam.response.success){
        ROS_INFO("get param ok");// %d value:", srv_setMode.response.success);
        cout << srv_getParam.response.value.integer << "   " << srv_getParam.response.value.real << endl;
    }else{
        ROS_ERROR("Failed get param");
        return -1;
    }


    return 0;
}
