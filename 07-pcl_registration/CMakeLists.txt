cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project(07-pcl_registration)

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

# 01_registration_ICP
add_executable(01_registration_ICP src/01_registration_ICP.cpp)
target_link_libraries(01_registration_ICP ${PCL_LIBRARIES})

# 02_registration_IncrementRegist
add_executable(02_registration_IncrementRegist src/02_registration_IncrementRegist.cpp)
target_link_libraries(02_registration_IncrementRegist ${PCL_LIBRARIES})
