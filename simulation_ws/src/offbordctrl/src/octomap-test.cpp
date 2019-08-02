#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <octomap_msgs/conversions.h>

using namespace std;
using namespace octomap;



ros::Publisher marker_pub;
double cur_x=0,cur_y=0,cur_z=0;

void publishpath()
{
    float f = 0.0;
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/world";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;



    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;



    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;



    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;



    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 100; ++i)
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = (int32_t)i - 50;
      p.y = y;
      p.z = z;

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }


    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);
}


void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

void current_location_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    // cout << msg->pose.position.x << endl;
    cur_x = msg->pose.position.x;
    cur_y = msg->pose.position.y;
    cur_z = msg->pose.position.z;

}

void binary_cb(const octomap_msgs::Octomap::ConstPtr& msg)
{
    cout<<"binary"<<endl;
    AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    
    //OcTree octree* = dynamic_cast<OcTree*>(tree);
    /*
    if (octree)
    { 
        point3d query (0., 0., 0.);
        OcTreeNode* result = octree->search (query);
        print_query_info(query, result);
        cout << "octree found \n";
    }
*/
//      point3d origin(-1.5, -1.5, -0.5);
//   point3d direction;
//   point3d ray_end;

  
//   for(float z = 0; z <= 0.25; z += 0.125){
//     direction = point3d(1, 1, z);
//     cout << endl;
//     cout << "casting ray from " << origin  << " in the " << direction << " direction"<< endl;
//     bool success = tree.castRay(origin, direction, ray_end);

//     if(success){
//       cout << "ray hit cell with center " << ray_end << endl;
      
//       point3d intersection;
//       success = tree.getRayIntersection(origin, direction, ray_end, intersection);
//       if(success)
//         cout << "entrance point is " << intersection << endl;
//     }
// }
}

//void full_cb(const octomap_msgs::Octomap::ConstPtr& msg){
//    cout<<"full";
//}



int main(int argc, char **argv)
{
    cout << "print starting octomap test node \n";
    ros::init(argc, argv, "octomap_test_node");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("/freepath", 10);

    ros::Subscriber binary_sub = nh.subscribe<octomap_msgs::Octomap>("octomap_binary", 1000, binary_cb);
    //ros::Subscriber full_sub = nh.subscriber<octomap_msgs::Octomap>("octomap_full", 1000,full_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, current_location_cb);

    ros::spin();
    return 0;

}
