#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;


int main(int argc, char **argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_projected(new pcl::PointCloud<PointT>);

    // Fill in the cloud data
    cloud->width = 50;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before projection: " << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
                  << cloud->points[i].y << " "
                  << cloud->points[i].z << std::endl;

    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    // Create the filtering object
    pcl::ProjectInliers<PointT> proj;   
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    std::cerr << "Cloud after projection: " << std::endl;
    for (size_t i = 0; i < cloud_projected->points.size(); ++i)
        std::cerr << "    " << cloud_projected->points[i].x << " "
                  << cloud_projected->points[i].y << " "
                  << cloud_projected->points[i].z << std::endl;

    // show cloud
    pcl::visualization::PCLVisualizer viewer("ProjectInliers Filter Demo");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler(cloud, 255, 255, 255); // white
    viewer.addPointCloud(cloud, cloud_color_handler, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_projected_color_handler(cloud_projected, 230, 20, 20); // Green
    viewer.addPointCloud(cloud_projected, cloud_projected_color_handler, "cloud_projected");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_projected");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    while (!viewer.wasStopped())
    {
        // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }

    return (0);
}