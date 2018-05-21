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


class point
{
    public:
        double x;
        double y;
};

point trajectory[5000];

double h=1;
double dx=1.5;
double dy=1.0;
double length=3.0;
double width=2.0;

double curr_x=0;
double curr_y=0;

double del_x = 0.1;
double del_y = 0.1;
double del_z = 0.1;

int indx_trajectory=0;

int indx_max;

int counter = 0;
int counter1 = 0;

string tgstatus="hold";

int setPointReached = 0;
int config = -1;
int homeLocation = -1;
double homeX = 0;
double homeY = 0;

bool homeLocationSet = false;
ros::Publisher current_status_pub;


void init_trajectory()
{
    int m=int(length/dx);
    int n=int(width/dy);
    int indx=0;
	int xmul=1,ymul=1;
	if((config==0&&homeLocation==2) || (config==1&&homeLocation==1) || (config==2&&homeLocation==0) || (config==3&&homeLocation==3))
	{
		xmul = -1;
		ymul = -1;
	}
	else if((config==0&&homeLocation==3) || (config==1&&homeLocation==2) || (config==2&&homeLocation==1) || (config==3&&homeLocation==0))
	{
		xmul = 1;
		ymul = -1;
	}
	else if((config==0&&homeLocation==0) || (config==1&&homeLocation==3) || (config==2&&homeLocation==2) || (config==3&&homeLocation==1))
	{
		xmul = 1;
		ymul = 1;
	}
	if((config==0&&homeLocation==1) || (config==1&&homeLocation==0) || (config==2&&homeLocation==3) || (config==3&&homeLocation==2))
	{
		xmul = -1;
		ymul = 1;
	}
    for(int i=0;i<n;i++)
    {
        if(i%2==0)
        {
            for(int j=0;j<m;j++)
            {
                trajectory[indx].x=homeX+xmul*j*dx;
                trajectory[indx].y=homeY+ymul*i*dy;
                indx++;
            }
        }
        else
        {
            for(int j=m-1;j>=0;j--)
            {
                trajectory[indx].x=homeX+xmul*j*dx;
                trajectory[indx].y=homeY+ymul*i*dy;
                indx++;
            }
        }
    }
	trajectory[indx].x=homeX;
    trajectory[indx].y=homeY;
	indx++;
    indx_max = indx;
	
	for(int i=0;i<indx_max;i++)
	{
		cout << trajectory[i].x << " " << trajectory[i].y << endl;
	}
}

void targetStatuscallback(const std_msgs::String::ConstPtr& status)
{
	ROS_INFO("Target Status: [%s]", status->data.c_str());
    tgstatus = status->data;
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

    if(!homeLocationSet && counter < 100)
	{
		homeX += cx;
		homeY += cy;
		counter++;
		if(counter == 100)
		{
			homeLocationSet = true;
			homeX /= counter;
			homeY /= counter;
			homeX = 0;
			homeY = 0;
			counter = 0;
		}
		return;
	}
	
    if(h!=0 && abs(h-cz)<del_z && abs(curr_x - cx)<del_x && abs(curr_y - cy)<del_y)
    {
		counter++;
		if(counter > 0 && counter1 == 0)
		{
			if(setPointReached == 0)
			{
				std_msgs::String stat;
				stat.data = "hold";
				current_status_pub.publish(stat);
				cout <<  "punlishing hold" << endl;
			}
			counter = 0;
			setPointReached = 1;
		}
		if(tgstatus == "found")
		{
			counter1++;
		}
		else if(tgstatus == "home")
		{
			indx_trajectory++;
		}
    }

	if(setPointReached == 1 && tgstatus == "done")
	{
        	indx_trajectory++;
		counter = 0;
		tgstatus="hold";
		setPointReached = 0;
		std_msgs::String stat;
		stat.data = "moving";
		current_status_pub.publish(stat);
		cout <<  "punlishing moving" << endl;
	}
	if(tgstatus == "found")
	{
		if(counter1 == 0)
		{
			counter1++;
			cout << "found" << endl;
			curr_x = cx;
			curr_y = cy;
			return;
		}
		else if(counter1>2)
		{
			indx_trajectory = indx_max - 1;
			tgstatus="home";
		}
		else
		{
			return;
		}
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

void get_config()
{
	ifstream inf("/home/intel1/ros_repo/ros_ws/src/offbordctrl/src/config.txt");
	if (!inf)
    {
        // Print an error and exit
        cerr << "Uh oh, config.txt could not be opened for reading!" << endl;
        exit(1);
    }
	int x1,y1,x2,y2,x3,y3,x4,y4;
	inf >> x1 >> y1 >> x2 >> y2 >>x3 >> y3 >>x4 >> y4;
	inf >> x1 >> y1;
	if(x2==1&&y2==0&&x3==1&&y3==1&&x4==0&&y4==1)
	{
		config = 0;
		if(x1==0&&y1==0)
		{
			homeLocation = 0;
		}
		else if(x1==1&&y1==0)
		{
			homeLocation = 1;
		}
		else if(x1==1&&y1==1)
		{
			homeLocation = 2;
		}
		else if(x1==0&&y1==1)
		{
			homeLocation = 3;
		}
	}
	else if(x2==0&&y2==1&&x3==-1&&y3==1&&x4==-1&&y4==0)
	{
		config = 1;
		if(x1==0&&y1==0)
		{
			homeLocation = 0;
		}
		else if(x1==0&&y1==1)
		{
			homeLocation = 1;
		}
		else if(x1==-1&&y1==1)
		{
			homeLocation = 2;
		}
		else if(x1==-1&&y1==0)
		{
			homeLocation = 3;
		}
	}
	else if(x2==-1&&y2==0&&x3==-1&&y3==-1&&x4==0&&y4==-1)
	{
		config = 2;
		if(x1==0&&y1==0)
		{
			homeLocation = 0;
		}
		else if(x1==-1&&y1==0)
		{
			homeLocation = 1;
		}
		else if(x1==-1&&y1==-1)
		{
			homeLocation = 2;
		}
		else if(x1==0&&y1==-1)
		{
			homeLocation = 3;
		}
	}
	else if(x2==0&&y2==-1&&x3==1&&y3==-1&&x4==1&&y4==0)
	{
		config = 3;
		if(x1==0&&y1==0)
		{
			homeLocation = 0;
		}
		else if(x1==0&&y1==-1)
		{
			homeLocation = 1;
		}
		else if(x1==1&&y1==-1)
		{
			homeLocation = 2;
		}
		else if(x1==1&&y1==0)
		{
			homeLocation = 3;
		}
	}
	cout << config << "  " << homeLocation << endl;
}


int main(int argc, char **argv)
{
	//init_trajectory();

    ros::init(argc, argv, "pub_trajectory_node");
    ros::NodeHandle nh;
	
	current_status_pub = nh.advertise<std_msgs::String>("pubTrajectory/currentStatus", 100);
    ros::Subscriber local_pos_sub = nh.subscribe("/mavros/local_position/pose",1000,localPoscallback);
	ros::Subscriber target_status_sub = nh.subscribe("/target/search/status",1000,targetStatuscallback);

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
    ros::Rate rate(150.0);
	//ros::Rate initrate(20.0);
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }
	while(ros::ok()&&!homeLocationSet)
	{
		ros::spinOnce();
        rate.sleep();
	}
	cout << "homex: " << homeX << " homey: " << homeY << endl;
	get_config();
    init_trajectory();

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

    //ROS_INFO("going to the first way point");
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
