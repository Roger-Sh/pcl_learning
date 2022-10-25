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

# 01_filter_PassThrough
add_executable(01_filter_PassThrough src/01_filter_PassThrough.cpp)
target_link_libraries(01_filter_PassThrough ${PCL_LIBRARIES})

# 02_filter_VoxelGrid
add_executable(02_filter_VoxelGrid src/02_filter_VoxelGrid.cpp)
target_link_libraries(02_filter_VoxelGrid ${PCL_LIBRARIES})

# 03_filter_StatisticalOutlierRemoval
add_executable(03_filter_StatisticalOutlierRemoval src/03_filter_StatisticalOutlierRemoval.cpp)
target_link_libraries(03_filter_StatisticalOutlierRemoval ${PCL_LIBRARIES})

# 04_filter_RadiusOutlierRemoval
add_executable(04_filter_RadiusOutlierRemoval src/04_filter_RadiusOutlierRemoval.cpp)
target_link_libraries(04_filter_RadiusOutlierRemoval ${PCL_LIBRARIES})

# 05_filter_ConditionalRemoval
add_executable(05_filter_ConditionalRemoval src/05_filter_ConditionalRemoval.cpp)
target_link_libraries(05_filter_ConditionalRemoval ${PCL_LIBRARIES})

# 06_filter_BilateralFilter
add_executable(06_filter_BilateralFilter src/06_filter_BilateralFilter.cpp)
target_link_libraries(06_filter_BilateralFilter ${PCL_LIBRARIES})

# 07_filter_ProjectInliers
add_executable(07_filter_ProjectInliers src/07_filter_ProjectInliers.cpp)
target_link_libraries(07_filter_ProjectInliers ${PCL_LIBRARIES})

# 08_filter_ExtractIndices
add_executable(08_filter_ExtractIndices src/08_filter_ExtractIndices.cpp)
target_link_libraries(08_filter_ExtractIndices ${PCL_LIBRARIES})
