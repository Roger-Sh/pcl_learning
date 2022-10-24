cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project(03-pcl_feature)

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

# 01_normal_estimation
add_executable(01_normal_estimation src/01_normal_estimation.cpp)
target_link_libraries(01_normal_estimation ${PCL_LIBRARIES})


# 02_normal_estimation_using_integral_images
add_executable(02_normal_estimation_using_integral_images src/02_normal_estimation_using_integral_images.cpp)
target_link_libraries(02_normal_estimation_using_integral_images ${PCL_LIBRARIES})
