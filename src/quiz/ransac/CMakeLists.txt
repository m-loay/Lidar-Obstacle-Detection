cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(playback)

# #set build type to debug and C++ standard to 11
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11) 
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

# Use simulation  testing mode or Unit testing
option (USE_RANSAC_2D "Use Ransac 2D Mode" OFF)
option (USE_RANSAC_3D "Use Ransac 3D Mode" ON)

#find the pacakges required for PCL
find_package(PCL 1.2 REQUIRED)

#Inclue and Link directores
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

##compile the source code and link it with external libraries

if(USE_RANSAC_2D)
    message("USE_RANSAC_2D")
    add_executable (quizRansac ransac2d.cpp ../../render/render.cpp)
elseif(USE_RANSAC_3D)
message("USE_RANSAC_3D")
    add_executable (quizRansac ransac3d.cpp ../../render/render.cpp)
endif()

target_link_libraries (quizRansac ${PCL_LIBRARIES})




