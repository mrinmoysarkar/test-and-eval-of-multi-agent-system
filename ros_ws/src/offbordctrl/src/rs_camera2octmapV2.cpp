#include <ros/ros.h>
#include <algorithm> 
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <librealsense2/rs.hpp>

//#include <example.hpp>
 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <boost/foreach.hpp>
#include <pcl_ros/point_cloud.h>

#include <stdio.h>
#include <math.h> 

using namespace std;

octomap::OcTree tree (0.1);

int counter=0;
rs2::pointcloud pc;
rs2::points points;
octomap::point3d origin (0, 0, 0);

/**
    Function to convert 2D pixel point to 3D point by extracting point
    from PointCloud2 corresponding to input pixel coordinate. This function
    can be used to get the X,Y,Z coordinates of a feature using an 
    RGBD camera, e.g., Kinect.
    */
    void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p)
    {
      // get width and height of 2D point cloud data
      int width = pCloud.width;
      int height = pCloud.height;

      // Convert from u (column / width), v (row/height) to position in array
      // where X,Y,Z data starts
      int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

      // compute position in array where x,y,z data start
      int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
      int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
      int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

      float X = 0.0;
      float Y = 0.0;
      float Z = 0.0;

      memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
      memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
      memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

     // put data into the point p
      p.x = X;
      p.y = Y;
      p.z = Z;

    }

void pointCloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
    /*if(counter < 10){
		counter++;
	} else if (counter==10){
		  	tree.writeBinary("simple_tree.bt");
  			cout << "wrote example file simple_tree.bt" << endl << endl;
  			cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  			cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl; 
	}*/

    //points = pc.calculate(*(msg));
    if(msg -> is_dense)
    {
        cout << "in cb \n";
    }
/*
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    
    BOOST_FOREACH (const pcl::PointXYZ& pt, temp_cloud->points)
    {
        //if(pt.x != )
            printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    }
*/
    //BOOST_FOREACH(const pcl::PointCloud)
    //tree.insertPointCloud(*(msg),origin);
    //cout << "in cb \n";


    geometry_msgs::Point point;
    int width = msg->width;
    int height = msg->height;
    cout << "width: " << width << " height: " << height << endl;
    bool insert_in_tree = false;
    octomap::Pointcloud octpc;
    for(int u = 0; u < width; u++)
    {
        for(int v = 0; v < height; v++)
        {
            pixelTo3DPoint(*msg, u, v, point);
            if(!isnan(point.x) && !isnan(point.y) && !isnan(point.y))
            {
                //cout << point.x << "  " << point.y << "  " << point.z << endl;
                octomap::point3d single_3d_point (point.x, point.y, point.z);
                octpc.push_back(single_3d_point);
                insert_in_tree = true;
            }
          }
    }
    if(insert_in_tree)
    {
        tree.insertPointCloud(octpc,origin);
        counter++;
        if(counter == 10)
        {
            tree.writeBinary("/home/intel-custom/ros-intel-uav-rpeo/ros_ws/simple_tree.bt");
            cout << "done writing in file" << endl;
        }
    }
}

int main(int argc, char * argv[])
{
    

	ros::init(argc, argv, "rs_camera2OctmapV2");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<sensor_msgs::PointCloud2>
            ("/camera/depth_registered/points", 10, pointCloud_cb);
/*
    for(int loop=0;loop<10;loop++)
    {
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        points = pc.calculate(depth);
        auto color = frames.get_color_frame();
        if (!color)
        {
            color = frames.get_infrared_frame();
        }
        pc.map_to(color);
        auto vertices = points.get_vertices();
        octomap::Pointcloud octpc;
        for (int i = 0; i < points.size(); ++i)
        {
            octomap::point3d single_3d_point (vertices[i].x, vertices[i].y, vertices[i].z);
            octpc.push_back(single_3d_point);
         }
        tree.insertPointCloud(octpc,origin);
    }
*/
    ros::spin();

    return 0;
}

