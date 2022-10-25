#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>

int main(int argc, char **argv)
{
    // load pcd file
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file");
        return (-1);
    }
    std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

    // Compute the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);
    // set search method with KdTree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::Normal>);
    normal_estimation.setRadiusSearch(0.03);
    normal_estimation.compute(*cloud_with_normals);

    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (cloud_with_normals);
    // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_vfh (new pcl::search::KdTree<pcl::PointXYZ> ());
    vfh.setSearchMethod (tree_vfh);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

    // Compute the features
    vfh.compute (*vfhs);

    // vfhs->size () should be of size 1*
    std::cout << "vfhs->size(): " << vfhs->size() << std::endl;
}