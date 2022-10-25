#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_inlier(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ>("../../pcd/table_scene_lms400.pcd", *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // build the condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cond->addComparison(
        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison(
        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 0.5)));

    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter(*cloud_filtered_inlier);

    // show cloud
    pcl::visualization::PCLVisualizer viewer("StatisticalOutlierRemoval Filter Demo");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 255, 255);
    viewer.addPointCloud(cloud, cloud_color_handler, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_filtered_inlier_color_handler(cloud_filtered_inlier, 230, 20, 20); // Red
    viewer.addPointCloud(cloud_filtered_inlier, cloud_filtered_inlier_color_handler, "cloud_filtered_inlier");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_filtered_inlier");
    while (!viewer.wasStopped())
    {
        // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }

    return (0);
}