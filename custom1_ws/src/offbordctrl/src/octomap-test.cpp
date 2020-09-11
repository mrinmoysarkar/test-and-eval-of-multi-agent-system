#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <string>

using namespace std;

void binary_cb(const octomap_msgs::Octomap::ConstPtr& msg){
    cout<<"binary";
}

//void full_cb(const octomap_msgs::Octomap::ConstPtr& msg){
//    cout<<"full";
//}

void loop(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_node");
    ros::NodeHandle nh;

    ros::Subscriber binary_sub = nh.subscribe<octomap_msgs::Octomap>
        ("octomap_binary", 1000, binary_cb);
    //ros::Subscriber full_sub = nh.subscriber<octomap_msgs::Octomap>
        //("octomap_full", 1000,full_cb);

    ros::spin();

}

int main(int argc, char **argv)
{

    return 0;

}
