#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/recognition/impl/implicit_shape_model.hpp>

/** CMD
../../06-pcl_recognition/bin/02_recognition_ImplicitShapeModel ism_train/ism_train_cat.pcd 0 ism_train/ism_train_horse.pcd 1 ism_train/ism_train_lioness.pcd 2 ism_train/ism_train_michael.pcd 3 ism_train/ism_train_wolf.pcd 4 ism_test/ism_test_cat.pcd 0
**/

int main(int argc, char **argv)
{
    // check args
    if (argc == 0 || argc % 2 == 0)
    {
        std::cout << "Usage example: \n";
        std::cout << "$ ./implicit_shape_model \n" 
            << "ism_train_cat.pcd 0 \n"
            << "ism_train_horse.pcd 1 \n"
            << "ism_train_lioness.pcd 2 \n"
            << "ism_train_michael.pcd 3 \n"
            << "ism_train_wolf.pcd 4 \n"
            << "ism_test_cat.pcd 0 \n";

        return (-1);
    }

    std::cout << "test0 \n";

    unsigned int number_of_training_clouds = (argc - 3) / 2;

    /**
     * @brief get normals and class for every train cloud
     * 
     */

    // normal estimator
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setRadiusSearch(25.0);

    // init clouds, normals, classes
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> training_clouds;
    std::vector<pcl::PointCloud<pcl::Normal>::Ptr> training_normals;
    std::vector<unsigned int> training_classes;

    // loop for every train clouds
    for (unsigned int i_cloud = 0; i_cloud < number_of_training_clouds - 1; i_cloud++)
    {
        // load train cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr tr_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[i_cloud * 2 + 1], *tr_cloud) == -1)
            return (-1);

        // get normals
        pcl::PointCloud<pcl::Normal>::Ptr tr_normals = (new pcl::PointCloud<pcl::Normal>)->makeShared();
        normal_estimator.setInputCloud(tr_cloud);
        normal_estimator.compute(*tr_normals);

        // get class
        unsigned int tr_class = static_cast<unsigned int>(strtol(argv[i_cloud * 2 + 2], 0, 10));

        training_clouds.push_back(tr_cloud);
        training_normals.push_back(tr_normals);
        training_classes.push_back(tr_class);
    }

    std::cout << "test1 \n";

    /**
     * @brief train ImplicitShapeModel
     * 
     */

    // init FPFH estimator
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153>>::Ptr fpfh(new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153>>);
    fpfh->setRadiusSearch(30.0);
    pcl::Feature<pcl::PointXYZ, pcl::Histogram<153>>::Ptr feature_estimator(fpfh);

    // set ImplicitShapeModelEstimation
    pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;
    ism.setFeatureEstimator(feature_estimator);
    ism.setTrainingClouds(training_clouds);
    ism.setTrainingNormals(training_normals);
    ism.setTrainingClasses(training_classes);
    ism.setSamplingSize(2.0f);

    // start train
    pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model = boost::shared_ptr<pcl::features::ISMModel>(new pcl::features::ISMModel);
    ism.trainISM(model);

    // save&load trained model
    std::string file("trained_ism_model.txt");
    model->saveModelToFile(file);
    model->loadModelFromfile(file);

    std::cout << "test2 \n";


    /**
     * @brief test ImplicitShapeModel
     * 
     */

    // load test cloud
    unsigned int testing_class = static_cast<unsigned int>(strtol(argv[argc - 1], 0, 10));
    pcl::PointCloud<pcl::PointXYZ>::Ptr testing_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[argc - 2], *testing_cloud) == -1)
        return (-1);

    // get test cloud normals
    pcl::PointCloud<pcl::Normal>::Ptr testing_normals = (new pcl::PointCloud<pcl::Normal>)->makeShared();
    normal_estimator.setInputCloud(testing_cloud);
    normal_estimator.compute(*testing_normals);

    // set ism for test cloud
    boost::shared_ptr<pcl::features::ISMVoteList<pcl::PointXYZ>> vote_list = ism.findObjects(
        model,
        testing_cloud,
        testing_normals,
        testing_class);

    // findStrongestPeaks 
    double radius = model->sigmas_[testing_class] * 10.0;
    double sigma = model->sigmas_[testing_class];
    std::vector<pcl::ISMPeak, Eigen::aligned_allocator<pcl::ISMPeak>> strongest_peaks;
    vote_list->findStrongestPeaks(strongest_peaks, testing_class, radius, sigma);

    std::cout << "test3 \n";


    /**
     * @brief visualize recognition results
     * 
     */

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
    colored_cloud->height = 0;
    colored_cloud->width = 1;

    // add original cloud 
    pcl::PointXYZRGB point;
    point.r = 255;
    point.g = 255;
    point.b = 255;
    for (size_t i_point = 0; i_point < testing_cloud->points.size(); i_point++)
    {
        point.x = testing_cloud->points[i_point].x;
        point.y = testing_cloud->points[i_point].y;
        point.z = testing_cloud->points[i_point].z;
        colored_cloud->points.push_back(point);
    }
    colored_cloud->height += testing_cloud->points.size();

    // add strongest_peaks cloud
    point.r = 255;
    point.g = 0;
    point.b = 0;
    for (size_t i_vote = 0; i_vote < strongest_peaks.size(); i_vote++)
    {
        point.x = strongest_peaks[i_vote].x;
        point.y = strongest_peaks[i_vote].y;
        point.z = strongest_peaks[i_vote].z;
        colored_cloud->points.push_back(point);
    }
    colored_cloud->height += strongest_peaks.size();

    pcl::visualization::CloudViewer viewer("Result viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped())
    {
    }

    std::cout << "test4 \n";


    return (0);
}