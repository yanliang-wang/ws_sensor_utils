cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_tools)
set(CMAKE_BUILD_TYPE "release")  #release debug 模式

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
        INCLUDE_DIRS include
        DEPENDS PCL
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

link_directories(
        include
        ${PCL_LIBRARY_DIRS}
)
add_library(get_sync_data include/get_sync_data/get_sync_data.cpp)

cs_add_executable(get_sync_data_node ./src/get_sync_data_node.cpp )
target_link_libraries(get_sync_data_node 
        get_sync_data       
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
)