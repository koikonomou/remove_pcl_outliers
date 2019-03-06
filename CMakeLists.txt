cmake_minimum_required(VERSION 2.8.3)
project(remove_pcl_outliers)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  roscpp
  sensor_msgs)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS})

add_executable(filtered_scene src/filtered_scene.cpp)
target_link_libraries(filtered_scene ${catkin_LIBRARIES})