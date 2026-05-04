cmake_minimum_required(VERSION 3.5)
project(map_tools_rviz_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)

set(HDR_FILES
  src/map_tools_panel.h
)

foreach(header "${HDR_FILES}")
  qt5_wrap_cpp(moc_files "${header}")
endforeach()

set(SRC_FILES
  src/map_tools_panel.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${HDR_FILES} ${moc_files})

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  rviz2
  std_msgs
  geometry_msgs
  nav_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${rviz_common}
  ${rviz_rendering}
  ${rviz_default_plugins}
  ${pluginlib}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  rviz2
  std_msgs
  geometry_msgs
  nav_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
