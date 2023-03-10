#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

int main()
{
    /**
     * @brief 与基于邻域的法线估计方法NormalEstimation相比，
     * 基于积分图的方法IntegralImageNormalEstimation，
     * 在同一份数据情况下，相对会快一些，但需注意，基于积分图的方法只能输入有序点云；
     */


    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../pcd/table_scene_mug_stereo_textured.pcd", *cloud);

    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    
    /**
     * @brief The COVARIANCE_MATRIX mode creates 9 integral images to compute the normal 
     * for a specific point from the covariance matrix of its local neighborhood. 
     * The AVERAGE_3D_GRADIENT mode creates 6 integral images to compute smoothed versions 
     * of horizontal and vertical 3D gradients and computes the normals using the 
     * cross-product between these two gradients. The AVERAGE_DEPTH_CHANGE mode creates 
     * only a single integral image and computes the normals from the average depth changes.
     */
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}