#include <cstdlib>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/Altitude.h>
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

//Global Variable
double xPose;
double yPose;
double zPose;
double xOrient;
double yOrient;
double zOrient;
double wOrient;
double vel_xLineTwist;
double vel_yLineTwist;
double vel_zLineTwist;
double vel_xAngTwist;
double vel_yAngTwist;
double vel_zAngTwist;
double imu_xOrient;
double imu_yOrient;
double imu_zOrient;
double imu_wOrient;
double imu_xAngVel;
double imu_yAngVel;
double imu_zAngVel;
double imu_xLinAcc;
double imu_yLinAcc;
double imu_zLinAcc;
double roll_angle;
double pitch_angle;
double yaw_angle;



mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
/*
void odom_cb(const nav_msgs::Odometry::ConstPtr & msg){
    xOrient = msg -> pose.pose.orientation.x;
    yOrient = msg -> pose.pose.orientation.y;
    zOrient = msg -> pose.pose.orientation.z;
    wOrient = msg -> pose.pose.orientation.w;
}
*/
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){
    xPose = msg->pose.position.x;
    yPose = msg->pose.position.y;
    zPose = msg->pose.position.z;
    double quatx= msg->pose.orientation.x;
    double quaty= msg->pose.orientation.y;
    double quatz= msg->pose.orientation.z;
    double quatw= msg->pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll_angle, pitch_angle, yaw_angle);
}

void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr & msg){
    vel_xLineTwist = msg -> twist.linear.x;
    vel_yLineTwist = msg -> twist.linear.y;
    vel_zLineTwist = msg -> twist.linear.z;
    vel_xAngTwist = msg -> twist.angular.x;
    vel_yAngTwist = msg -> twist.angular.y;
    vel_zAngTwist = msg -> twist.angular.z;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr & msg){
    imu_xOrient = msg -> orientation.x;
    imu_yOrient = msg -> orientation.y;
    imu_zOrient = msg -> orientation.z;
    imu_wOrient = msg -> orientation.w;
    imu_xAngVel = msg -> angular_velocity.x;
    imu_yAngVel = msg -> angular_velocity.y;
    imu_zAngVel = msg -> angular_velocity.z;
    imu_xLinAcc = msg -> linear_acceleration.x;
    imu_yLinAcc = msg -> linear_acceleration.y;
    imu_zLinAcc = msg -> linear_acceleration.z;
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

    ros::init(argc, argv, "flight_path_node");
    ros::NodeHandle nh;

    //Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::Subscriber position_sub = nh.subscribe("mavros/local_position/pose", 100, pose_cb);

   // ros::Subscriber orientation_sub = nh.subscribe("mavros/local_position/odom", 100, odom_cb);

    ros::Subscriber velocity_sub = nh.subscribe("/mavros/local_position/velocity",100, velocity_cb);

    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data",100,imu_cb);

    //Publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 100);

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

    double secs =ros::Time::now().toSec();
    long sec = long(secs);

    std::string time = std::to_string(sec);
    std::string fileName = "flight_path_data" + time + ".csv";

    cout<<fileName<<endl;



    //Creating text file onto the Home Page
    ofstream file;
    file.open(fileName);

    //file << "[xPose]   [yPose]   [zPose]     [Roll]   [Pitch]   [Yaw]   [xLineTwist] [yLineTwist][zLineTwist] [xAngTwist] [yAngTwist] [zAngTwist] [imu_xAngVel] [imu_yAngVel] [imu_zAngVel] [imu_xLinAcc] [imu_yLinAcc] [imu_zLinAcc] [Time]\n";

    int i = 0;

    while(true){

        uint64_t seconds =ros::Time::now().toNSec();
        //long second = long(seconds);

        //Hold
        if(i<=5*20){
                if (i==1){
                    ROS_INFO("HOLD");
                }
          pose.pose.position.x = 0;
          pose.pose.position.y = 0;
          pose.pose.position.z = 0;
          i++;
        }

        // Takeoff
        else if (i >= 5*20 && i <= 10*20){
            if (i==5*20+1){
                ROS_INFO("TAKEOFF");
            }
           //cout <<"Taking Off"<<endl;
           pose.pose.position.x = 0;
           pose.pose.position.y = 0;
           pose.pose.position.z = 1;
           i++;
       }

        // State 1
        else if(i >= 10*20 && i <= 15*20){
            if (i==10*20+1){
                ROS_INFO("GOING TO STATE 1");
            }
           //cout <<"Going to State1"<<endl;
           pose.pose.position.x = 3;
           pose.pose.position.y = 0;
           pose.pose.position.z = 1;
           i++;
        }
        // State 2
        else if(i >= 15*20 && i <= 20*20){
            if (i==15*20+1){
                ROS_INFO("GOING TO STATE 2");
            }
           //cout <<"Going to State2"<<endl;
           pose.pose.position.x = 3;
           pose.pose.position.y = 3;
           pose.pose.position.z = 1;
           i++;
        }
        // State 3
        else if(i >= 20*20 && i <= 25*20){
            if (i==20*20+1){
                ROS_INFO("GOING TO STATE 3");
            }
           //cout <<"Going to State3"<<endl;
           pose.pose.position.x = 0;
           pose.pose.position.y = 3;
           pose.pose.position.z = 1;
           i++;
        }
        // State 4
        else if(i >= 25*20 && i <= 30*20){
            if (i==25*20+1){
                ROS_INFO("GOING TO STATE 4");
            }
           //cout <<"Going to State4"<<endl;
           pose.pose.position.x = 0;
           pose.pose.position.y = 6;
           pose.pose.position.z = 1;
           i++;
        }
        // State 5
        else if(i >= 30*20 && i <= 35*20){
            if (i==(30*20)+1){
                ROS_INFO("GOING TO STATE 5");
            }
           //cout <<"Going to State5"<<endl;
           pose.pose.position.x = 3;
           pose.pose.position.y = 6;
           pose.pose.position.z = 1;
           i++;
        }

        // Home
        else if(i >= 35*20 && i <= 40*20){
            if (i==35*20+1){
                ROS_INFO("GOING BACK TO ORIGINAL POSITION");
            }
            //cout <<"Going home"<<endl;
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 1;
            i++;
        }

        //Landing
        else if(i >= 40*20 && i <= 45*20){
                if (i==40*20+1){
                    ROS_INFO("tring to land");
                    land_client.call(land_cmd);
                }
            i++;
        }

        //Holding
        else if(i >= 45*20 && i <= 50*20){
            if (i==45*20+1){
                ROS_INFO("HOLDING");
            }
            i++;
        }

        // Break while loop
        else {
           break;
        }

      //Writing data onto text file
      file << seconds << "," << xPose << ", " << yPose << ", " << zPose << ", "
           << roll_angle << ", " << pitch_angle << ", " << yaw_angle << ", "
           << vel_xLineTwist << ", " << vel_yLineTwist << ", " << vel_zLineTwist << ", " << vel_xAngTwist << ", " << vel_yAngTwist << ", "<< vel_zAngTwist << ", "
           << imu_xAngVel << ", " << imu_yAngVel << ", " << imu_zAngVel << ", "
           << imu_xLinAcc << ", " << imu_yLinAcc << ", " << imu_zLinAcc << endl;

      //Publishing position
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
      //cout<< "i: "<<i<<endl;
    } // end while loop
/*
    //Start landing command
    ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      ROS_INFO("tring to land");
      ros::spinOnce();
      rate.sleep();
    }
*/
    file.close();
    return 0;
}
