#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;

/**
 * @brief show segmentation result
 *
 * @param cloud_p, segmented planar cloud
 * @param cloud_f, rest filtered cloud
 */
void show_segmentation_results(
    const pcl::PointCloud<PointT>::Ptr cloud_p,
    const pcl::PointCloud<PointT>::Ptr cloud_f)
{
    pcl::visualization::PCLVisualizer viewer("ExtractIndices Filter Demo");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_p_color_handler(cloud_p, 255, 255, 255); // white
    viewer.addPointCloud(cloud_p, cloud_p_color_handler, "cloud_p");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_p");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_f_color_handler(cloud_f, 230, 20, 20); // Red
    viewer.addPointCloud(cloud_f, cloud_f_color_handler, "cloud_f");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_f");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    while (!viewer.wasStopped())
    {
        // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }
}

int main(int argc, char **argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    // pcl::PointCloud<PointT>::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>); // segmented plane
    pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>); // plane segmentation filtered cloud

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read("../../pcd/table_scene_lms400.pcd", *cloud);
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // set Segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // segmentation loop
    int i = 0, nr_points = (int)cloud_filtered->points.size();
    // While 30% of the original cloud is still there
    // while (cloud_filtered->points.size() > 0.3 * nr_points)

    while(i < 6)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);

        // show extract plane, rest cloud
        show_segmentation_results(cloud_p, cloud_f);

        // check break condition
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            // break;
        }

        // swap segmentation filtered cloud
        cloud_filtered.swap(cloud_f);
        i++;
    }

    return (0);
}