cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(playback)

# #set build type to debug and C++ standard to 11
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11) 
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

# Use simulation  testing mode or Unit testing
option (USE_RAY "Use Ray Mode" OFF)       
option (USE_POINT_CLOUD "Use Point Cloud Mode" OFF)
option (USE_SEG "Use segmentation Mode" OFF)
option (USE_PCL_CLUSTER "Use Pcl Cluster Mode" OFF)
option (USE_PCL_FILTER "Use Pcl Cluster Mode" OFF)
option (USE_FINAL_PROJECT "Use Final Project Mode" OFF)
option (USE_FINAL_PROJECT_SELF "USE_FINAL_PROJECT_SELF" ON)


# configure a header file to pass some of the CMake settings
# to the source code
make_directory("${PROJECT_SOURCE_DIR}/src/_generated")
configure_file (
  "${PROJECT_SOURCE_DIR}/LidarConfig.h.in"
  "${PROJECT_SOURCE_DIR}/src/_generated/LidarConfig.h"
  )

#find the pacakges required for PCL
   
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	find_package(PCL 1.2 REQUIRED)
else()
	find_package(PCL 1.10 REQUIRED)
endif()

#Inclue and Link directores
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

##compile the source code and link it with external libraries
add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})




