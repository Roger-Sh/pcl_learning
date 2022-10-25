cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project(04-pcl_visualization)

# set cmake
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# find PCL
find_package(PCL 1.9.1 REQUIRED
    PATHS "/home/shan/App/pcl/pcl-pcl-1.9.1/build/installed"
    NO_DEFAULT_PATH
)

# include and link PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 01_simple_pcd_viewer
add_executable(01_simple_pcd_viewer src/01_simple_pcd_viewer.cpp)
target_link_libraries(01_simple_pcd_viewer ${PCL_LIBRARIES})

# 02_multithread_pcd_viewer
add_executable(02_multithread_pcd_viewer src/02_multithread_pcd_viewer.cpp)
target_link_libraries(02_multithread_pcd_viewer ${PCL_LIBRARIES})

# 03_range_image_viewer
add_executable(03_range_image_viewer src/03_range_image_viewer.cpp)
target_link_libraries(03_range_image_viewer ${PCL_LIBRARIES})

# 04_pcl_visualizer
add_executable(04_pcl_visualizer src/04_pcl_visualizer.cpp)
target_link_libraries(04_pcl_visualizer ${PCL_LIBRARIES})
