#include <ros/ros.h>
#include <librealsense2/rs.hpp> 
#include <algorithm> 
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

using namespace std;

int main(int argc, char * argv[]) try
{
    rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline pipe;
    pipe.start();
    octomap::OcTree tree (0.1);
    octomap::point3d origin (0, 0, 0);

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
  tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl; 
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
