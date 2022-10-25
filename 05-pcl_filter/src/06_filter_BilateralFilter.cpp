#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZI PointT;

int main(int argc, char *argv[])
{
    /**
     * @brief TODO bilateral filter is for organized cloud, try to use organized cloud data to test
     * 
     */


    std::string incloudfile = "../../pcd/table_scene_lms400_outliers.pcd";
    float sigma_s = 10.0;
    float sigma_r = 1.0;

    // read pcd file
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(incloudfile.c_str(), *cloud);

    // KdTree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    // filter
    // pcl::PointCloud<PointT> cloud_filtered;
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::BilateralFilter<PointT> bf;
    bf.setInputCloud(cloud);
    bf.setSearchMethod(tree);
    bf.setHalfSize(sigma_s);
    bf.setStdDev(sigma_r);
    bf.filter(*cloud_filtered);

    // show cloud
    pcl::visualization::PCLVisualizer viewer("BilateralFilter Filter Demo");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler(cloud, 255, 255, 255); // Red
    viewer.addPointCloud(cloud, cloud_color_handler, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered_color_handler(cloud_filtered, 230, 20, 20); // Green
    viewer.addPointCloud(cloud_filtered, cloud_filtered_color_handler, "cloud_filtered");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_filtered");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    while (!viewer.wasStopped())
    {
        // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }

    return (0);
}