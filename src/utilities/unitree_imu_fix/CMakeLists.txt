cmake_minimum_required(VERSION 3.5)
project(unitree_imu_fix)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)

add_executable(utlidar_imu_quat_fix src/utlidar_imu_quat_fix.cpp)
ament_target_dependencies(utlidar_imu_quat_fix rclcpp sensor_msgs)

add_executable(lowstate_imu_fix src/lowstate_imu_fix.cpp)
ament_target_dependencies(lowstate_imu_fix rclcpp sensor_msgs unitree_go)

add_executable(lf_imu_fix src/lf_imu_fix.cpp)
ament_target_dependencies(lf_imu_fix rclcpp sensor_msgs unitree_go)

add_executable(unitree_clock_sync src/unitree_clock_sync.cpp)
ament_target_dependencies(unitree_clock_sync rclcpp sensor_msgs unitree_go)

install(TARGETS utlidar_imu_quat_fix lowstate_imu_fix lf_imu_fix unitree_clock_sync
  DESTINATION lib/${PROJECT_NAME})

ament_package()
