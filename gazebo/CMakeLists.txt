cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
add_compile_options(-std=c++11)

# ROS
find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${my_robot_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugins
add_library(my_robot_plugin SHARED my_robot_plugin.cc)
target_link_libraries(my_robot_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(filter_plugin SHARED filter_plugin.cc)
target_link_libraries(filter_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

