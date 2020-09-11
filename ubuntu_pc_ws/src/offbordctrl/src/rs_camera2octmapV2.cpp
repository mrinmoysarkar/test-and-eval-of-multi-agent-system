#include <ros/ros.h>
#include <algorithm> 
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

using namespace std;

octomap::OcTree tree (0.1);

int counter=0;

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
    if(counter < 10){
		counter++;
	} else if (counter==10){
		  	tree.writeBinary("simple_tree.bt");
  			cout << "wrote example file simple_tree.bt" << endl << endl;
  			cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  			cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl; 
	}
}

int main(int argc, char * argv[])
{

    
    octomap::point3d origin (0, 0, 0);

	ros::init(argc, argv, "rs_camera2OctmapV2");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<sensor_msgs::PointCloud2>
            ("/camera/depth/points", 10, pointCloud_cb);

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


    return 0;
}

