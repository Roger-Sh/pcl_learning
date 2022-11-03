cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project(pcl_learning)

# set cmake
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(01-basic_usage)
add_subdirectory(02-advanced_usage)
add_subdirectory(03-pcl_feature)
add_subdirectory(04-pcl_visualization)
add_subdirectory(05-pcl_filter)
add_subdirectory(06-pcl_recognition)
add_subdirectory(07-pcl_registration)