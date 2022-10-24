#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

/**
 * @brief normal estimation with all points in dataset
 *
 * @param cloud
 */
void estimateNormal_allPoints(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = cloud.makeShared();

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_ptr);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*cloud_normals);

    // cloud_normals->size () should have the same size as the input cloud->size ()
    std::cout << "etimateNormal_allPoints: cloud_normals size: " <<  cloud_normals->size() << std::endl;
}

/**
 * @brief normal estimation with subset of dataset using indices
 *
 * @param cloud
 */
void estimateNormal_setIndices(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = cloud.makeShared();

    // Create a set of indices to be used. For simplicity, we're going to be using the first 10% of the points in cloud
    std::vector<int> indices(std::floor(cloud_ptr->size() / 10));
    for (std::size_t i = 0; i < indices.size(); ++i)
        indices[i] = i;

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_ptr);

    // Pass the indices
    pcl::IndicesPtr indicesptr(new std::vector<int>(indices));
    ne.setIndices(indicesptr);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*cloud_normals);

    // cloud_normals->size () should have the same size as the input indicesptr->size ()
    std::cout << "etimateNormal_setIndices: cloud_normals size: " <<  cloud_normals->size() << std::endl;
}

/**
 * @brief normal estimation with search surface
 *
 * @param cloud
 */
void estimateNormal_setSearchSurface(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    // orignial cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = cloud.makeShared();

    // downsampled using voxelgrid
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> filter_voxelgrid;
    filter_voxelgrid.setInputCloud(cloud_ptr);
    filter_voxelgrid.setLeafSize(10, 10, 10);
    filter_voxelgrid.filter(*cloud_filtered_ptr);
    std::cout << "estimateNormal_setIndices: cloud_ptr size: " <<  cloud_ptr->size() << std::endl;
    std::cout << "estimateNormal_setIndices: cloud_filtered_ptr size: " <<  cloud_filtered_ptr->size() << std::endl;

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_filtered_ptr);

    // Pass the original data (before downsampling) as the search surface
    ne.setSearchSurface (cloud_ptr);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given surface dataset.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);

    // cloud_normals->size () should have the same size as the input cloud_downsampled->size ()
    std::cout << "estimateNormal_setIndices: cloud_normals size: " <<  cloud_normals->size() << std::endl;
}

int main()
{
    // create a cubic cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 1000;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);
    for (auto &point : cloud)
    {
        point.x = 100.0 * rand() / (RAND_MAX + 1.0f);
        point.y = 100.0 * rand() / (RAND_MAX + 1.0f);
        point.z = 100.0 * rand() / (RAND_MAX + 1.0f);
    }

    estimateNormal_allPoints(cloud);
    estimateNormal_setIndices(cloud);
    estimateNormal_setSearchSurface(cloud);

    return 0;
}