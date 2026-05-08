cmake_minimum_required(VERSION 3.5)
project(utlidar_obstacle_layer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(cloud_merger src/cloud_merger.cpp)
ament_target_dependencies(cloud_merger
  rclcpp sensor_msgs nav_msgs pcl_conversions pcl_ros)

add_executable(utlidar_to_baselink src/utlidar_to_baselink.cpp)
ament_target_dependencies(utlidar_to_baselink
  rclcpp sensor_msgs pcl_conversions pcl_ros tf2_ros geometry_msgs)

install(TARGETS cloud_merger utlidar_to_baselink
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
