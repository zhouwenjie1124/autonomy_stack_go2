cmake_minimum_required(VERSION 3.5)
project(go2w_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf dae config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
