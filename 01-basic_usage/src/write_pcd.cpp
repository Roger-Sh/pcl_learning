#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width = 100;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);

    for (auto &point : cloud)
    {
        point.x = 10.0 * rand() / (RAND_MAX + 1.0f);
        point.y = 10.0 * rand() / (RAND_MAX + 1.0f);
        point.z = 5.0 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;

    for (const auto &point : cloud)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    return (0);
}