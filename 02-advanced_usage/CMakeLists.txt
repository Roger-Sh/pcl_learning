cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project(02-advanced_usage)

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

include_directories(
    include)

# 01_adding_custom_ptype.cpp
add_executable(01_adding_custom_ptype src/01_adding_custom_ptype.cpp)
target_link_libraries(01_adding_custom_ptype ${PCL_LIBRARIES})

# bilateral
# add_executable(bilateral src/bilateral.cpp)
# target_link_libraries(bilateral ${PCL_LIBRARIES})

