#include <iostream>
#include <unistd.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

#include <pcl/visualization/cloud_viewer.h>

/**
 * @brief // run pcl viewer once
 * 
 * @param viewer 
 */
void viewerOnce(pcl::visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(0.8, 0.8, 0.8);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.25, "sphere", 0);
    std::cout << "run pcl viewer once" << std::endl;
}

/**
 * @brief // run pcl viewer once per loop
 * 
 * @param viewer 
 */
int user_data;
void viewerOncePerLoop(pcl::visualization::PCLVisualizer &viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "run pcl viewer once per loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);

    // FIXME: possible race condition here:
    user_data++;
    std::cout << "user_data from viewerOncePerLoop: " << user_data << std::endl;
    usleep(500000);

}

/**
 * @brief This function displays the help
 * 
 * @param program_name 
 */
void showHelp(char *program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

/**
 * @brief load pcd file with template pcl point type
 * 
 * @tparam PointT 
 * @param argc 
 * @param argv 
 * @return pcl::PointCloud<PointT>::Ptr 
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr load_pcd_file(int argc, char **argv)
{
    // Show help
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        showHelp(argv[0]);
        exit(-1);
    }

    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

    if (filenames.size() != 1)
    {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

        if (filenames.size() != 1)
        {
            showHelp(argv[0]);
            exit(-1);
        }
        else
        {
            file_is_pcd = true;
        }
    }

    // Load file | Works with PCD and PLY files
    typename pcl::PointCloud<PointT>::Ptr source_cloud(new typename pcl::PointCloud<PointT>());

    if (file_is_pcd) // load PCD
    {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp(argv[0]);
            exit(-1);
        }
    }
    else    // load PLY
    {
        if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp(argv[0]);
            exit(-1);
        }
    }

    return source_cloud;
}

/**
 * @brief enter c to resume
 * 
 */
void pause_c()
{
    std::cout << "enter c to continue" << std::endl;

    std::string str;
    while (std::getline(std::cin, str))
    {
        if (str[0] != 'c')
        {
            std::cout << "enter c to continue" << std::endl;
            continue;
        }
        else
        {
            std::cout << "continue" << std::endl;
            break;
        }
    }
}


/**
 * @brief main
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    // load pcd
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    cloud = load_pcd_file<pcl::PointXYZRGBA>(argc, argv);


    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    // blocks until the cloud is actually rendered
    viewer.showCloud(cloud);

    pause_c();

    // use the following functions to get access to the underlying more advanced/powerful
    // PCLVisualizer

    // This will only get called once
    viewer.runOnVisualizationThreadOnce(viewerOnce);

    // This will get called once per visualization iteration
    viewer.runOnVisualizationThread(viewerOncePerLoop);
    while (!viewer.wasStopped())
    {
        // pause_c();
        usleep(1000000);

        // you can also do cool processing here
        // FIXME: Note that this is running in a separate thread from viewerPsycho
        // and you should guard against race conditions yourself...
        user_data++;
        std::cout << "user_data from main: " << user_data << std::endl;
    }
    return 0;
}