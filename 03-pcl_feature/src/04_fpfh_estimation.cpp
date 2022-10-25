#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
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

    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(cloud_with_normals);
    // alternatively, if cloud is of type PointNormal, do fpfh.setInputNormals (cloud);
    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh(new pcl::search::KdTree<pcl::PointXYZ>);
    fpfh.setSearchMethod(tree_fpfh);
    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch(0.05);

    // Compute the features
    fpfh.compute(*fpfhs);

    // fpfhs->size () should have the same size as the input cloud->size ()*
    std::cout << "output points.size (): " << fpfhs->points.size() << std::endl;

    pcl::FPFHSignature33 descriptor = fpfhs->points[0];
    std::cout << descriptor << std::endl;
}