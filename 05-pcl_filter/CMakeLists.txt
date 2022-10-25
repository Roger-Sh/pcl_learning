cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project(05-pcl_filter)

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

# 01_filter_passThrough
add_executable(01_filter_passThrough src/01_filter_passThrough.cpp)
target_link_libraries(01_filter_passThrough ${PCL_LIBRARIES})

# 02_filter_voxelGrid
add_executable(02_filter_voxelGrid src/02_filter_voxelGrid.cpp)
target_link_libraries(02_filter_voxelGrid ${PCL_LIBRARIES})

