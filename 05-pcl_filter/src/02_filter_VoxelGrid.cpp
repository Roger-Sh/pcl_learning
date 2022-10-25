#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read("../../pcd/table_scene_lms400.pcd", *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " 
        << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ").";

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

    // show cloud
    pcl::visualization::PCLVisualizer viewer("VoxelGrid Filter Demo");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 255, 255);
    viewer.addPointCloud(cloud, cloud_color_handler, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_filtered_color_handler(cloud_filtered, 230, 20, 20); // Red
    viewer.addPointCloud(cloud_filtered, cloud_filtered_color_handler, "cloud_filtered");
    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_filtered");
    while (!viewer.wasStopped())
    { 
        // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }

    return (0);
}